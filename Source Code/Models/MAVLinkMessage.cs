using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using FlightLogConverter.ViewModels;

namespace FlightLogConverter.Models
{
    /*
        DISCLAIMER: The source code is influenced by the Mission Planner application. 
            We do not claim it as purely our idea and do not intend to credit to ourselves only. 
            We modified and input our own ideas, but it does not consist of ONLY our ideas.
    */
    
    public class MAVLinkMessage
    {
        public static readonly MAVLinkMessage Invalid = new MAVLinkMessage();

        public MAVLinkMessage()
        {
            this.rxtime = DateTime.MinValue;
        }

        public MAVLinkMessage(byte[] buffer): this(buffer, DateTime.UtcNow)
        { }

        public MAVLinkMessage(byte[] buffer, DateTime rxTime)
        {
            this.buffer = buffer;
            this.rxtime = rxTime;

            processBuffer(buffer);
        }

        object _locker = new object();
        public DateTime rxtime { get; set; }
        public byte header { get; internal set; }
        public byte payloadlength { get; internal set; }
        public byte incompat_flags { get; internal set; }
        public byte compat_flags { get; internal set; }
        public byte seq { get; internal set; }
        public byte sysid { get; internal set; }
        public byte compid { get; internal set; }
        public uint msgid { get; internal set; }
        public ushort crc16 { get; internal set; }
        public byte[] sig { get; internal set; }
        public byte sigLinkid
        {
            get
            {
                if (sig != null)
                    return sig[0];

                return 0;
            }
        }

        public ulong sigTimeStamp
        {
            get
            {
                if (sig != null)
                {
                    byte[] temp = new byte[8];
                    Array.Copy(sig, 1, temp, 0, 6);
                    return BitConverter.ToUInt64(temp, 0);
                }
                return 0;
            }
        }

        public int Length
        {
            get
            {
                if (buffer == null) return 0;
                return buffer.Length;
            }
        }

        public bool ismavlink2
        {
            get
            {
                if (buffer != null && buffer.Length > 0)
                    return (buffer[0] == GlobalAppData.MAVLINK_STX);

                return false;
            }
        }

        object _data;
        public object data
        {
            get
            {
                lock (_locker)
                {
                    if (_data != null)
                        return _data;
                    _data = Activator.CreateInstance(GlobalAppData.GetMessageInfo(msgid).type);

                    try
                    {
                        if (ismavlink2)
                            GlobalAppData.ByteArrayToStructure(buffer, ref _data, GlobalAppData.MAVLINK_NUM_HEADER_BYTES, payloadlength);
                        else
                            GlobalAppData.ByteArrayToStructure(buffer, ref _data, 6, payloadlength);
                    }
                    catch (Exception ex)
                    {

                    }
                }
                return _data;
            }
        }


        public T ToStructure<T>()
        {
            return (T)data;
        }

        private byte[] _buffer;
        public byte[] buffer
        {
            get { return _buffer; }
            set
            {
                _buffer = value;
                processBuffer(_buffer);
            }
        }

        internal void processBuffer(byte[] buffer)
        {
            _data = null;
            if (buffer[0] == GlobalAppData.MAVLINK_STX)
            {
                header = buffer[0];
                payloadlength = buffer[1];
                incompat_flags = buffer[2];
                compat_flags = buffer[3];
                seq = buffer[4];
                sysid = buffer[5];
                compid = buffer[6];
                msgid = (uint)((buffer[9] << 16) + (buffer[8] << 8) + buffer[7]);

                var crc1 = GlobalAppData.MAVLINK_CORE_HEADER_LEN + payloadlength + 1;
                var crc2 = GlobalAppData.MAVLINK_CORE_HEADER_LEN + payloadlength + 2;

                crc16 = (ushort)((buffer[crc2] << 8) + buffer[crc1]);

                if ((incompat_flags & GlobalAppData.MAVLINK_IFLAG_SIGNED) > 0)
                {
                    sig = new byte[GlobalAppData.MAVLINK_SIGNATURE_BLOCK_LEN];
                    Array.ConstrainedCopy(buffer, buffer.Length - GlobalAppData.MAVLINK_SIGNATURE_BLOCK_LEN, sig, 0,
                        GlobalAppData.MAVLINK_SIGNATURE_BLOCK_LEN);
                }
            }
            else
            {
                header = buffer[0];
                payloadlength = buffer[1];
                seq = buffer[2];
                sysid = buffer[3];
                compid = buffer[4];
                msgid = buffer[5];

                var crc1 = GlobalAppData.MAVLINK_CORE_HEADER_MAVLINK1_LEN + payloadlength + 1;
                var crc2 = GlobalAppData.MAVLINK_CORE_HEADER_MAVLINK1_LEN + payloadlength + 2;

                crc16 = (ushort)((buffer[crc2] << 8) + buffer[crc1]);
            }
        }
    }
}
