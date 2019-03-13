using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using FlightLogConverter.ViewModels;

namespace FlightLogConverter.Models
{
    public class PacketExtractor : IDisposable
    {
        #region Properties
        private bool _logreadmode = false;
        public bool logreadmode
        {
            get { return _logreadmode; }
            set { _logreadmode = value; }
        }

        public BufferedStream rawlogfile { get; set; }
        volatile object readlock = new object();
        //private readonly Subject<int> _bytesReceivedSubj = new Subject<int>();
        DateTime _bpstime { get; set; }
        int _mavlink1count = 0;
        int _mavlink2count = 0;
        int _mavlink2signed = 0;
        int _bps1 = 0;
        int _bps2 = 0;

        string buildplaintxtline = "";
        internal string plaintxtline = "";
        private BinaryReader _logplaybackfile;
        public BinaryReader logplaybackfile

        {
            get { return _logplaybackfile; }
            set
            {
                _logplaybackfile = value;
            }
        }
        public class MavlinkCRC
        {
            const int X25_INIT_CRC = 0xffff;
            const int X25_VALIDATE_CRC = 0xf0b8;

            public static ushort crc_accumulate(byte b, ushort crc)
            {
                unchecked
                {
                    byte ch = (byte)(b ^ (byte)(crc & 0x00ff));
                    ch = (byte)(ch ^ (ch << 4));
                    return (ushort)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
                }
            }

            public static ushort crc_calculate(byte[] pBuffer, int length)
            {
                if (length < 1)
                {
                    return 0xffff;
                }
                // For a "message" of length bytes contained in the unsigned char array
                // pointed to by pBuffer, calculate the CRC
                // crcCalculate(unsigned char* pBuffer, int length, unsigned short* checkConst) < not needed

                ushort crcTmp;
                int i;

                crcTmp = X25_INIT_CRC;

                for (i = 1; i < length; i++) // skips header
                {
                    crcTmp = crc_accumulate(pBuffer[i], crcTmp);
                    //Console.WriteLine(crcTmp + " " + pBuffer[i] + " " + length);
                }

                return (crcTmp);
            }

        }


        #endregion
        public PacketExtractor()
        {
        }

        private ICommsSerial _BaseStream;
        public ICommsSerial BaseStream
        {
            get { return _BaseStream; }
            set
            {
                // This is called every time user changes the port selection, so we need to make sure we cleanup
                // any previous objects so we don't leave the cleanup of system resources to the garbage collector.
                if (_BaseStream != null)
                {
                    try
                    {
                        if (_BaseStream.IsOpen)
                        {
                            _BaseStream.Close();
                        }
                    }
                    catch { }
                    IDisposable dsp = _BaseStream as IDisposable;
                    if (dsp != null)
                    {
                        try
                        {
                            dsp.Dispose();
                        }
                        catch { }
                    }
                }
                _BaseStream = value;
            }
        }

        public MAVLinkMessage ReadPacket()
        {
            byte[] buffer = new byte[GlobalAppData.MAX_PACKET_LEN + 25];
            int count = 0;
            int length = 0;
            int readcount = 0;
            MAVLinkMessage message = null;

            DateTime start = DateTime.Now;

            lock (readlock)
            {
                while ((BaseStream != null && BaseStream.IsOpen) || logreadmode)
                {
                    try
                    {
                        if (readcount > 300)
                        {
                            break;
                        }

                        readcount++;
                        if (logreadmode)
                        {
                            message = ReadLog();
                            buffer = message.buffer;
                            if (buffer == null || buffer.Length == 0)
                                return MAVLinkMessage.Invalid;
                        }
                        else
                        {
                            if (BaseStream.ReadTimeout != 1200)
                                BaseStream.ReadTimeout = 1200; // 1200 ms between chars - the gps detection requires this.

                            DateTime to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                            while (BaseStream.IsOpen && BaseStream.BytesToRead <= 0)
                            {
                                if (DateTime.Now > to)
                                {
                                    throw new TimeoutException("Timeout");
                                }

                                Task.Delay(1);
                            }
                            if (BaseStream.IsOpen)
                            {
                                BaseStream.Read(buffer, count, 1);
                                if (rawlogfile != null && rawlogfile.CanWrite)
                                    rawlogfile.WriteByte(buffer[count]);
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        break;
                    }

                    // check if looks like a mavlink packet and check for exclusions and write to console
                    if (buffer[0] != 0xfe && buffer[0] != 'U' && buffer[0] != 0xfd)
                    {
                        if (buffer[0] >= 0x20 && buffer[0] <= 127 || buffer[0] == '\n' || buffer[0] == '\r')
                        {
                            // check for line termination
                            if (buffer[0] == '\r' || buffer[0] == '\n')
                            {
                                // check new line is valid
                                if (buildplaintxtline.Length > 3)
                                    plaintxtline = buildplaintxtline;

                                // reset for next line
                                buildplaintxtline = "";
                            }

                            TCPConsole.Write(buffer[0]);
                            buildplaintxtline += (char)buffer[0];
                        }

                        //_bytesReceivedSubj.OnNext(1);
                        count = 0;
                        buffer[1] = 0;
                        continue;
                    }

                    // reset count on valid packet
                    readcount = 0;
                    // check for a header
                    if (buffer[0] == 0xfe || buffer[0] == 0xfd || buffer[0] == 'U')
                    {
                        var mavlinkv2 = buffer[0] == GlobalAppData.MAVLINK_STX ? true : false;

                        int headerlength = mavlinkv2 ? GlobalAppData.MAVLINK_CORE_HEADER_LEN : GlobalAppData.MAVLINK_CORE_HEADER_MAVLINK1_LEN;
                        int headerlengthstx = headerlength + 1;

                        // if we have the header, and no other chars, get the length and packet identifiers
                        if (count == 0 && !logreadmode)
                        {
                            DateTime to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                            while (BaseStream.IsOpen && BaseStream.BytesToRead < headerlength)
                            {
                                if (DateTime.Now > to)
                                {
                                    throw new TimeoutException("Timeout");
                                }

                                Task.Delay(1);
                            }

                            int read = BaseStream.Read(buffer, 1, headerlength);
                            count = read;
                            if (rawlogfile != null && rawlogfile.CanWrite)
                                rawlogfile.Write(buffer, 1, read);
                        }

                        // packet length
                        if (buffer[0] == GlobalAppData.MAVLINK_STX)
                        {
                            length = buffer[1] + headerlengthstx +
                                     GlobalAppData.MAVLINK_NUM_CHECKSUM_BYTES; // data + header + checksum - magic - length
                            if ((buffer[2] & GlobalAppData.MAVLINK_IFLAG_SIGNED) > 0)
                            {
                                length += GlobalAppData.MAVLINK_SIGNATURE_BLOCK_LEN;
                            }
                        }
                        else
                        {
                            length = buffer[1] + headerlengthstx +
                                     GlobalAppData.MAVLINK_NUM_CHECKSUM_BYTES; // data + header + checksum - U - length    
                        }

                        if (count >= headerlength || logreadmode)
                        {
                            try
                            {
                                if (logreadmode)
                                {
                                }
                                else
                                {
                                    DateTime to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                                    while (BaseStream.IsOpen && BaseStream.BytesToRead < (length - (headerlengthstx)))
                                    {
                                        if (DateTime.Now > to)
                                        {
                                            break;
                                        }

                                        Task.Delay(1);
                                    }

                                    if (BaseStream.IsOpen)
                                    {
                                        int read = BaseStream.Read(buffer, headerlengthstx, length - (headerlengthstx));
                                        if (rawlogfile != null && rawlogfile.CanWrite)
                                        {
                                            // write only what we read, temp is the whole packet, so 6-end
                                            rawlogfile.Write(buffer, headerlengthstx, read);
                                        }
                                    }
                                }

                                count = length;
                            }
                            catch
                            {
                                break;
                            }
                            break;
                        }
                    }
                    count++;
                    if (count == 299)
                        break;
                }
            } // end readlock

            // resize the packet to the correct length
            Array.Resize<byte>(ref buffer, count);

            // add byte count
            //_bytesReceivedSubj.OnNext(buffer.Length);

            // update bps statistics
            if (_bpstime.Second != DateTime.Now.Second)
            {
                long btr = 0;
                if (BaseStream != null && BaseStream.IsOpen)
                {
                    btr = BaseStream.BytesToRead;
                }
                else if (logreadmode)
                {
                    btr = logplaybackfile.BaseStream.Length - logplaybackfile.BaseStream.Position;
                }
                _bps2 = _bps1; // prev sec
                _bps1 = 0; // current sec
                _bpstime = DateTime.Now;
                _mavlink1count = 0;
                _mavlink2count = 0;
                _mavlink2signed = 0;
            }

            _bps1 += buffer.Length;

            if (buffer.Length == 0)
                return MAVLinkMessage.Invalid;

            if (message == null)
                message = new MAVLinkMessage(buffer, DateTime.UtcNow);

            uint msgid = message.msgid;
            GlobalAppData.message_info msginfo = GlobalAppData.GetMessageInfo(msgid);

            // calc crc
            var sigsize = (message.sig != null) ? GlobalAppData.MAVLINK_SIGNATURE_BLOCK_LEN : 0;
            ushort crc = MavlinkCRC.crc_calculate(buffer, message.Length - sigsize - GlobalAppData.MAVLINK_NUM_CHECKSUM_BYTES);

            // calc extra bit of crc for mavlink 1.0/2.0
            if (message.header == 0xfe || message.header == 0xfd)
            {
                crc = MavlinkCRC.crc_accumulate(msginfo.crc, crc);
            }

            // check message length size vs table (mavlink1 explicit size check | mavlink2 allow all, undersize 0 trimmed, and oversize unknown extension)
            if (!message.ismavlink2 && message.payloadlength != msginfo.minlength)
            {
                if (msginfo.length == 0) // pass for unknown packets
                {
                }
                else
                {
                    return MAVLinkMessage.Invalid;
                }
            }

            // check crc
            if ((message.crc16 >> 8) != (crc >> 8) ||
                (message.crc16 & 0xff) != (crc & 0xff))
            {
                return MAVLinkMessage.Invalid;
            }


            byte sysid = message.sysid;
            byte compid = message.compid;
            byte packetSeqNo = message.seq;

            // stat count
            if (message.buffer[0] == GlobalAppData.MAVLINK_STX)
                _mavlink2count++;
            else if (message.buffer[0] == GlobalAppData.MAVLINK_STX_MAVLINK1)
                _mavlink1count++;

            // if its a gcs packet - dont process further
            if (buffer.Length >= 5 && (sysid == 255 || sysid == 253) && logreadmode) // gcs packet
            {
                return message;
            }

            return message;

        }

        public DateTime lastlogread {get; set;}
        Dictionary<Stream, Tuple<string, long>> streamfncache = new Dictionary<Stream, Tuple<string, long>>();

        private MAVLinkMessage ReadLog()
        {
            byte[] datearray = new byte[8];
            bool missingtimestamp = false;

            if (logplaybackfile.BaseStream is FileStream)
            {
                if (!streamfncache.ContainsKey(_logplaybackfile.BaseStream))
                    streamfncache[_logplaybackfile.BaseStream] = new Tuple<string, long>(((FileStream)_logplaybackfile.BaseStream).Name.ToLower(), logplaybackfile.BaseStream.Length);

                if (streamfncache[_logplaybackfile.BaseStream].Item1.EndsWith(".rlog"))
                    missingtimestamp = true;
            }

            else
            {
                if (!streamfncache.ContainsKey(_logplaybackfile.BaseStream))
                    streamfncache[_logplaybackfile.BaseStream] = new Tuple<string, long>("", logplaybackfile.BaseStream.Length);
            }

            if (!missingtimestamp)
            {
                int tem = logplaybackfile.BaseStream.Read(datearray, 0, datearray.Length);

                Array.Reverse(datearray);

                DateTime date1 = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

                UInt64 dateint = BitConverter.ToUInt64(datearray, 0);

                try
                {
                    // array is reversed above
                    if (datearray[7] == 254 || datearray[7] == 253)
                    {
                        //rewind 8bytes
                        logplaybackfile.BaseStream.Seek(-8, SeekOrigin.Current);
                    }
                    else
                    {
                        if ((dateint / 1000 / 1000 / 60 / 60) < 9999999)
                        {
                            date1 = date1.AddMilliseconds(dateint / 1000);

                            lastlogread = date1.ToLocalTime();
                        }
                    }
                }
                catch
                {
                }

            }

            byte[] temp = new byte[0];

            byte byte0 = 0;
            byte byte1 = 0;
            byte byte2 = 0;

            var filelength = streamfncache[_logplaybackfile.BaseStream].Item2;
            var filepos = logplaybackfile.BaseStream.Position;

            if (filelength == filepos)
                return MAVLinkMessage.Invalid;

            int length = 5;
            int a = 0;
            while (a < length)
            {
                if (filelength == filepos)
                    return MAVLinkMessage.Invalid;

                var tempb = (byte)logplaybackfile.ReadByte();
                filepos++;

                switch (a)
                {
                    case 0:
                        byte0 = tempb;
                        if (byte0 != 'U' && byte0 != GlobalAppData.MAVLINK_STX_MAVLINK1 && byte0 != GlobalAppData.MAVLINK_STX)
                        {
                            // seek to next valid
                            do
                            {
                                byte0 = logplaybackfile.ReadByte();
                            }
                            while (byte0 != 'U' && byte0 != GlobalAppData.MAVLINK_STX_MAVLINK1 && byte0 != GlobalAppData.MAVLINK_STX);
                            a = 1;
                            continue;
                        }
                        break;
                    case 1:
                        byte1 = tempb;
                        // handle length
                        {
                            int headerlength = byte0 == GlobalAppData.MAVLINK_STX ? 9 : 5;
                            int headerlengthstx = headerlength + 1;

                            length = byte1 + headerlengthstx + 2; // header + 2 checksum
                        }
                        break;
                    case 2:
                        byte2 = tempb;
                        // handle signing and mavlink2
                        if (byte0 == GlobalAppData.MAVLINK_STX)
                        {
                            if ((byte2 & GlobalAppData.MAVLINK_IFLAG_SIGNED) > 0)
                                length += GlobalAppData.MAVLINK_SIGNATURE_BLOCK_LEN;
                        }
                        // handle rest
                        {
                            temp = new byte[length];
                            temp[0] = byte0;
                            temp[1] = byte1;
                            temp[2] = byte2;

                            var readto = a + 1;
                            var readlength = length - (a + 1);
                            logplaybackfile.Read(temp, readto, readlength);
                            a = length;
                        }
                        break;
                }

                a++;
            }
            MAVLinkMessage tmp = new MAVLinkMessage(temp, lastlogread);

            //MAVlist[tmp.sysid, tmp.compid].cs.datetime = lastlogread;

            return tmp;
        }

        /// <summary>
        /// Print entire decoded packet to console
        /// </summary>
        /// <param name="datin">packet byte array</param>
        /// <returns>struct of data</returns>
        public object DebugPacket(MAVLinkMessage datin, ref string text, bool PrintToConsole, string delimeter = " ")
        {
            string textoutput = "";
            try
            {
                if (datin.Length > 5)
                {
                    textoutput =
                        string.Format(
                            "{0,2:X}{8}{1,2:X}{8}{2,2:X}{8}{3,2:X}{8}{4,2:X}{8}{5,2:X}{8}{6,2:X}{8}{7,6:X}{8}",
                            datin.header,
                            datin.payloadlength, datin.incompat_flags, datin.compat_flags, datin.seq, datin.sysid,
                            datin.compid, datin.msgid, delimeter);

                    object data = datin.data;

                    Type test = data.GetType();

                    {
                        textoutput = textoutput + test.Name + delimeter;

                        foreach (var field in test.GetFields())
                        {
                            // field.Name has the field's name.

                            object fieldValue = field.GetValue(data); // Get value

                            if (field.FieldType.IsArray)
                            {
                                textoutput = textoutput + field.Name + delimeter;
                                if (fieldValue.GetType() == typeof(byte[]))
                                {
                                    try
                                    {
                                        byte[] crap = (byte[])fieldValue;

                                        foreach (byte fiel in crap)
                                        {
                                            if (fiel == 0)
                                            {
                                                break;
                                            }
                                            else
                                            {
                                                textoutput = textoutput + (char)fiel;
                                            }
                                        }
                                    }
                                    catch
                                    {
                                    }
                                }
                                if (fieldValue.GetType() == typeof(short[]))
                                {
                                    try
                                    {
                                        short[] crap = (short[])fieldValue;

                                        foreach (short fiel in crap)
                                        {
                                            if (fiel == 0)
                                            {
                                                break;
                                            }
                                            else
                                            {
                                                textoutput = textoutput + Convert.ToString(fiel, 16) + "|";
                                            }
                                        }
                                    }
                                    catch
                                    {
                                    }
                                }
                                textoutput = textoutput + delimeter;
                            }
                            else
                            {
                                textoutput = textoutput + field.Name + delimeter + fieldValue.ToString() + delimeter;
                            }
                        }
                        var sig = "";
                        if (datin.sig != null)
                            sig = Convert.ToBase64String(datin.sig);

                        textoutput = textoutput + delimeter + "sig " + sig + delimeter + "Len" + delimeter + datin.Length + "\r\n";
                        if (PrintToConsole)
                            Console.Write(textoutput);

                        if (text != null)
                            text = textoutput;
                    }

                    return data;
                }
            }
            catch
            {
                textoutput = textoutput + "\r\n";
            }

            return null;
        }

        public void Dispose()
        {
        }
    }
}