using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections;
using System.Globalization;

namespace FlightLogConverter.Models
{
    public class BINParser
    {
        private ArrayList CollectedMessages;
        private ArrayList TempCollector;
        private ArrayList TimeCollector;
        private ArrayList TimeArray;

        private int GMSLoc = 3;
        private int GWkLoc = 4;
        private bool NotFound = true;
        private DateTime MainTime;
        private double TimeOffset = 18000.00;
        private int TimeIndex = 0;
        private double MainTimeUS = 0.0;


        public BINParser()
        {
            CollectedMessages = new ArrayList();
            TempCollector = new ArrayList();
            TimeArray = new ArrayList();
            TimeCollector = new ArrayList();
        }

        public void InputContent(string content)
        {
            CollectedMessages.Add(content);
        }
        
        public void ParseBIN()
        {
            InitialParse();
            TimeParse();
            FinalTimeParse();
        }

        private void InitialParse()
        {
            //Removing unnecessary labels
            int total = CollectedMessages.Count;
            for (int i = 1; i < total; i++)
            {
                string[] SplitData = ((string)CollectedMessages[i]).Split(new char[] { ',' });
                string EdittedMessage = "";
                int maxLength = SplitData.Length;
                for (int x = 0; x < maxLength; x++)
                {
                    if (String.Equals(SplitData[0], "FMT") && (x == 0 || x == 1 || x == 2 || x == 4))
                    {
                        continue;
                    }
                    else
                    {
                        EdittedMessage = EdittedMessage + SplitData[x] + ", ";
                    }
                }
                EdittedMessage = EdittedMessage.Substring(0, EdittedMessage.Length - 4);
                TempCollector.Add(EdittedMessage);
            }
        }

        private void TimeParse()
        {
            int TimeTotal = TempCollector.Count;

            for (int i = 0; i < TimeTotal; i++)
            {
                string[] SplitMessage = ((string)TempCollector[i]).Split(new char[] { ',' });
                string TimeMessage = "";
                if (Int32.TryParse(SplitMessage[1], out int TimeUS))
                {
                    string Time = (TimeUS * .000001).ToString("0.000000");
                    if (String.Equals(SplitMessage[0], "GPS") && NotFound)
                    {
                        MainTimeUS = (TimeUS * .000001);
                        MainTime = GetDate(Int32.Parse(SplitMessage[GWkLoc]), 1).AddMilliseconds((Int32.Parse(SplitMessage[GMSLoc]) - TimeOffset));
                        TimeIndex = i;
                        Time = MainTime.ToString("yyyy-MM-dd-HH:mm:ss.fff", CultureInfo.InvariantCulture);
                        NotFound = false;
                    }
                    TimeMessage = Time + ", " + String.Join(", ", SplitMessage);
                    TimeCollector.Add(i);
                }
                else
                {
                    TimeMessage = "No GPS TIME, " + String.Join(", ", SplitMessage);
                }
                TimeArray.Add(TimeMessage);
            }
        }

        private void FinalTimeParse()
        {
            int total = TimeCollector.Count;
            if (!NotFound)
            {
                for (int i = 0; i < total; i++)
                {
                    if (i == TimeIndex)
                        continue;

                    int TimeArrayIndex = (int)TimeCollector[i];
                    string[] SplitTime = ((string)TimeArray[TimeArrayIndex]).Split(new char[] { ',' });
                    if (Double.TryParse(SplitTime[0], out double ChangeTime))
                    {
                        double result = ChangeTime - MainTimeUS;
                        SplitTime[0] = MainTime.AddSeconds(result).ToString("yyyy-MM-dd-HH:mm:ss.fff", CultureInfo.InvariantCulture);
                        TimeArray[TimeArrayIndex] = String.Join(", ", SplitTime);
                    }
                }
            }
            else
            {
                for (int i = 0; i < total; i++)
                {
                    int TimeArrayIndex = (int)TimeCollector[i];
                    string[] SplitTime = ((string)TimeArray[TimeArrayIndex]).Split(new char[] { ',' });
                    SplitTime[0] = "NO GPS TIME";
                    TimeArray[TimeArrayIndex] = String.Join(", ", SplitTime);
                }
            }
        }

        private DateTime GetDate(int WeekNumber, int WeekDay)
        {
            DateTime jan6 = new DateTime(1980, 1, 6);
            jan6 = jan6.AddDays(WeekNumber * 7);
            return jan6;
        }

        public ArrayList GetMessages()
        {
            return TimeArray;
        }

        public void PrintOutputMessages()
        {
            foreach (string data in TimeArray)
                System.Diagnostics.Debug.WriteLine(data);
        }

    }
}

