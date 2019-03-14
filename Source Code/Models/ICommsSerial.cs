using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.IO;

namespace FlightLogConverter.Models
{
    /*
        DISCLAIMER: The source code is influenced by the Mission Planner application. 
            We do not claim it as purely our idea and do not intend to credit to ourselves only. 
            We modified and input our own ideas, but it does not consist of ONLY our ideas.
    */
    
    public interface ICommsSerial
    {
        // from serialport class
        // Methods
        void Close();
        void DiscardInBuffer();
        //void DiscardOutBuffer();
        void Open();
        int Read(byte[] buffer, int offset, int count);
        //int Read(char[] buffer, int offset, int count);
        int ReadByte();
        int ReadChar();
        string ReadExisting();
        string ReadLine();
        //string ReadTo(string value);
        void Write(string text);
        void Write(byte[] buffer, int offset, int count);
        //void Write(char[] buffer, int offset, int count);
        void WriteLine(string text);

        void toggleDTR();

        // Properties
        Stream BaseStream { get; }
        int BaudRate { get; set; }
        //bool BreakState { get; set; }
        int BytesToRead { get; }
        int BytesToWrite { get; }
        //bool CDHolding { get; }
        //bool CtsHolding { get; }
        int DataBits { get; set; }
        //bool DiscardNull { get; set; }
        //bool DsrHolding { get; }
        bool DtrEnable { get; set; }
        //Encoding Encoding { get; set; }
        //Handshake Handshake { get; set; }
        bool IsOpen { get; }
        //string NewLine { get; set; }
        Parity Parity { get; set; }
        //byte ParityReplace { get; set; }
        string PortName { get; set; }
        int ReadBufferSize { get; set; }
        int ReadTimeout { get; set; }
        bool RtsEnable { get; set; }
        StopBits StopBits { get; set; }
        int WriteBufferSize { get; set; }
        int WriteTimeout { get; set; }
    }
}
