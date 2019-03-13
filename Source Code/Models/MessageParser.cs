using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections;
using System.Diagnostics;

namespace FlightLogConverter.Models
{
    public class MessageParser
    {
        private ArrayList allMessages;
        private ArrayList parsedMessages;
        private ArrayList allHeaders;
        private string date;
        private string time;
        private bool isChanged;

        public MessageParser()
        {
            allMessages = new ArrayList();
            parsedMessages = new ArrayList();
            parsedMessages.Add("advanced mavlink package status,date,time");
            allHeaders = new ArrayList();
            date = "";
            time = "";
            isChanged = false;
        }

        public void addMessage(string message)
        {
            allMessages.Add(message);
        }

        public void parseMessages()
        {
            foreach (string message in allMessages)
            {
                string newMessage = " ";
                string header = " ";
                string[] splitMessage = split(message.Split(new char[] { ' ', ',' }));

                if (string.Equals(splitMessage[0], "*"))
                {
                    date = splitMessage[1];
                    time = splitMessage[2];
                    isChanged = true;
                }
                else
                {
                    newMessage = " ";
                    header = splitMessage[8];
                    if (header.Length > 7)
                        header = header.Remove(0, 8);
                    int indexInfo = 9;
                    while (indexInfo < splitMessage.Length - 3)
                    {
                        if (checkHeader(header, splitMessage[indexInfo]))
                        {
                            string newHeader = header + "__" + splitMessage[indexInfo];
                            string value = "";

                            if (splitMessage[indexInfo] == "voltages" || splitMessage[indexInfo] == "q" || splitMessage[indexInfo] == "middleware_custom_version")
                                value = "";
                            else if (splitMessage[indexInfo] == "text")
                            {
                                for (int i = indexInfo + 1; i < splitMessage.Length; i++)
                                {
                                    if (splitMessage[i] == "sig")
                                    {
                                        indexInfo = i;
                                        break;
                                    }
                                    else
                                    {
                                        value = value + " " + splitMessage[i];
                                    }
                                }
                            }
                            else
                                value = splitMessage[indexInfo + 1];
                            newMessage = composeMessage(newHeader, value, newMessage);
                        }
                        if (splitMessage[indexInfo] == "voltages" || splitMessage[indexInfo] == "q" || splitMessage[indexInfo] == "middleware_custom_version")
                        {
                            indexInfo++;
                        }
                        else
                            indexInfo = indexInfo + 2;
                    }
                }
                if (newMessage.Length > 3)
                {
                    if (isChanged)
                    {
                        isChanged = false;
                        newMessage = " , " + date + ", " + time + ", " + newMessage.Remove(0, 6);
                        
                    }
                    parsedMessages.Add(newMessage);
                }
            }
        }

        private string[] split(string[] messageParts)
        {
            ArrayList newMessageParts = new ArrayList();
            foreach(string item in messageParts)
            {
                if (item != string.Empty)
                    newMessageParts.Add(item);
            }
            return (string[]) newMessageParts.ToArray(typeof(string));
        }

        private bool checkHeader(string header, string label) //Header: Autopilot_version_t has a problem.
        {
            if (header.Length > 3 && label != "sig")
            {
                string newHeader = header + "__" + label;
                if (!allHeaders.Contains(newHeader))
                {
                    parsedMessages[0] = parsedMessages[0] + "," + newHeader;
                    allHeaders.Add(newHeader);
                }
                return true;

            }
            return false;
        }

        private string composeMessage(string headerName, string value, string currentString)
        {
            if (allHeaders.Contains(headerName) && currentString == " ")
            {
                int position = allHeaders.IndexOf(headerName) + 3;
                for (int i = 0; i < position; i++)
                {
                    currentString = currentString + ", ";
                }
            }
            currentString = currentString + value + ", ";
            return currentString;

        }

        public ArrayList getArray()
        {
            return parsedMessages;
        }

        public ArrayList GetHeaders()
        {
            allHeaders.Sort();
            return allHeaders;
        }
    }

}
