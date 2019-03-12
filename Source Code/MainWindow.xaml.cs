using System;
using System.Windows;
using FlightLogConverter.ViewModels;
using System.Windows.Forms;
using FlightLogConverter.Models;
using System.IO;
using System.Collections;
using System.Windows.Threading;
using System.Resources;
using System.Globalization;
using System.Reflection;
using System.Diagnostics;

namespace FlightLogConverter
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private GlobalAppData GAP;
        private ResourceManager RM;
        private MessageParser Parser;
        private StreamWriter sw;
        private BinaryLog BL;
        private BINParser BP;

        public MainWindow()
        {
            InitializeComponent();
            GAP = new GlobalAppData();
            RM = new ResourceManager("FlightLogConverter.Properties.TlogLabels", Assembly.GetExecutingAssembly());
            Parser = new MessageParser();
            this.DataContext = GAP;
            BL = new BinaryLog();
            BP = new BINParser();
        }

        private void LoadFile(object sender, EventArgs e)
        {
            using (OpenFileDialog ofdFile = new OpenFileDialog())
            {
                ofdFile.Filter = GAP.filemask;
                ofdFile.FilterIndex = 2;
                ofdFile.RestoreDirectory = true;

                if (ofdFile.ShowDialog() == System.Windows.Forms.DialogResult.OK)
                {
                    GAP.AllTlogData.Clear();
                    foreach (string logfile in ofdFile.FileNames)
                    {
                        string extension = Path.GetExtension(logfile);
                        
                        if (String.Equals(extension.ToLower(), ".tlog"))
                        {
                            progressbar1.Visibility = Visibility.Visible;
                            GAP.FilePath = Path.GetDirectoryName(logfile) + Path.DirectorySeparatorChar + Path.GetFileNameWithoutExtension(logfile) + ".csv";
                            GAP.FileName = Path.GetFileName(logfile);
                            using (PacketExtractor mine = new PacketExtractor())
                            {
                                try
                                {
                                    mine.logplaybackfile =
                                        new BinaryReader(File.Open(logfile, FileMode.Open, FileAccess.Read, FileShare.Read));
                                }
                                catch (Exception ex)
                                {
                                    return;
                                }
                                mine.logreadmode = true;

                                while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length)
                                {
                                    int percent =
                                        (int)
                                            ((float)mine.logplaybackfile.BaseStream.Position /
                                             (float)mine.logplaybackfile.BaseStream.Length * 100.0f);
                                    if (progressbar1.Value != percent)
                                    {
                                        progressbar1.Dispatcher.Invoke(() => progressbar1.Value = percent, DispatcherPriority.Background);
                                    }

                                    MAVLinkMessage packet = mine.ReadPacket();
                                    string text = "";
                                    mine.DebugPacket(packet, ref text, false, ",");
                                    Parser.addMessage("* " + mine.lastlogread.ToString("yyyy-MM-dd HH:mm:ss.fff"));
                                    Parser.addMessage(text);

                                }
                                Parser.parseMessages();
                                ArrayList headers = Parser.GetHeaders();

                                foreach (string header in headers)
                                {
                                    GlobalAppData.TlogData data = new GlobalAppData.TlogData();
                                    data.HeaderName = header;
                                    data.Name = RM.GetString(header);
                                    GAP.AddTlogData(data);
                                }

                                progressbar1.Value = 100;

                                EnableSettings();
                                FileLabel.Visibility = Visibility.Visible;

                                mine.logreadmode = false;
                                mine.logplaybackfile.Close();
                                mine.logplaybackfile = null;
                            }
                        }
                        else if (String.Equals(extension.ToLower(), ".bin"))
                        {
                            DialogResult Result = System.Windows.Forms.MessageBox.Show("The .bin file does not display any data parameters." + Environment.NewLine
                                + "We will be exporting the .bin file into a .csv file. Are you fine with that?", ".Bin Message", System.Windows.Forms.MessageBoxButtons.YesNo);

                            if (Result == System.Windows.Forms.DialogResult.Yes)
                            {
                                try
                                {
                                    using (BinaryReader br = new BinaryReader(new BufferedStream(File.OpenRead(logfile), 1024 * 1024)))
                                    {
                                        DateTime displaytimer = DateTime.MinValue;
                                        var length = br.BaseStream.Length;
                                        while (br.BaseStream.Position < length)
                                        {
                                            string data = BL.ReadMessage(br.BaseStream, length);
                                            BP.InputContent(data);
                                        }
                                    }
                                    BP.ParseBIN();
                                    ArrayList OutputMessages = BP.GetMessages();
                                    if (sw != null)
                                        sw.Dispose();

                                    sw = new StreamWriter(Path.GetDirectoryName(logfile) + Path.DirectorySeparatorChar + Path.GetFileNameWithoutExtension(logfile) + ".csv");
                                    foreach (string message in OutputMessages)
                                    {
                                        sw.WriteLine(message);
                                    }
                                    sw.Close();
                                    System.Windows.Forms.MessageBox.Show("You have successfully converted " + Path.GetFileName(logfile) + " into a .csv file");
                                    DisableSettings();
                                }
                                catch
                                {
                                    System.Windows.MessageBox.Show("You currently have the .csv open. Please close your .csv file and retry.", "Open File Error");
                                }
                            }                            
                        }
                        else
                        {
                            System.Windows.MessageBox.Show("You've selected the wrong file type.", "Wrong File Type Error");
                        }
                    }
                }
            }
        }

        private void ConvertToCsv(object sender, EventArgs e)
        {
            try
            {
                sw = new StreamWriter(GAP.FilePath);
            }
            catch
            {
                System.Windows.MessageBox.Show("You currently have the .csv open. Please close your .csv file and select the .CSV export option again.", "Open File Error");
                return;
            }
            progressbar1.Visibility = Visibility.Hidden;
            ArrayList messages = Parser.getArray();
            string NewHeader = (string)messages[0];
            ArrayList headers = new ArrayList();
            ArrayList indices = new ArrayList();
            if (!GAP.SelectingAll)
            {
                headers.AddRange(((string)messages[0]).Split(new char[] { ',' }));
                NewHeader = (string)headers[1] + "," + (string)headers[2];
                indices.Add(1);
                indices.Add(2);
                foreach (GlobalAppData.TlogData dataHeader in GAP.DataSelection)
                {
                    int index = headers.IndexOf(dataHeader.HeaderName);
                    if (index != -1)
                    {
                        indices.Add(index);
                        NewHeader += ", " + dataHeader.HeaderName;
                    }
                }
            }
            sw.WriteLine(NewHeader);
            for (int i = 1; i < messages.Count; i++)
            {
                if (!GAP.SelectingAll)
                {
                    string NewMessage = "";
                    string[] splittingData = ((string)messages[i]).Split(new char[] { ',' });
                    int count = 0;
                    foreach (int index in indices)
                    {
                        if (index < splittingData.Length)
                        {
                            string info = splittingData[index];
                            NewMessage += splittingData[index] + ", ";
                            if (info != " ")
                            {
                                count++;
                            }
                        }
                        else
                            NewMessage += ", ";
                    }
                    if (count > 2)
                        sw.WriteLine(NewMessage);
                }
                else
                    sw.WriteLine((string)messages[i]);
            }
            sw.Close();

            System.Windows.MessageBox.Show("You have successfully converted " + GAP.FileName + " into a .csv file");
            ClearLoadedLog(sender, e);
        }

        private void AddSelected(object sender, EventArgs e)
        {
            if (dgData.SelectedItems.Count > 0)
            {
                GlobalAppData.TlogData item = (GlobalAppData.TlogData)dgData.SelectedItem;
                item.Checked = true;
                if (!GAP.DataSelection.Contains(item))
                    GAP.DataSelection.Add(item);
            }
        }

        private void RemoveSelected(object sender, EventArgs e)
        {
            if (dgData.SelectedItems.Count > 0)
            {
                GlobalAppData.TlogData item = (GlobalAppData.TlogData)dgData.SelectedItem;
                item.Checked = false;
                if (GAP.DataSelection.Contains(item))
                    GAP.DataSelection.Remove(item);
            }
        }

        private void SelectAllData(object sender, EventArgs e)
        {
            foreach (var item in GAP.AllTlogData)
                item.Checked = true;

            dgData.Items.Refresh();
            GAP.SelectingAll = true;
        }

        private void RemoveAllData(object sender, EventArgs e)
        {
            foreach (var item in GAP.AllTlogData)
                item.Checked = false;

            GAP.DataSelection.Clear();
            dgData.Items.Refresh();
            GAP.SelectingAll = false;
        }

        private void ClearLoadedLog(object sender, EventArgs e)
        {
            GAP.AllTlogData.Clear();
            DisableSettings();
            Parser = new MessageParser();
            dgData.Items.Refresh();
            
        }

        private void EnableSettings()
        {
            ClearButton.IsEnabled = true;
            RemoveButton.IsEnabled = true;
            SelectButton.IsEnabled = true;
            CSVButton.IsEnabled = true;
            GAP.LoadedStatus = "File Loaded: ";
        }

        private void DisableSettings()
        {
            if (sw != null)
                sw.Dispose();
            GAP.LoadedStatus = "No File Loaded";
            FileLabel.Visibility = Visibility.Hidden;
            RemoveButton.IsEnabled = false;
            CSVButton.IsEnabled = false;
            SelectButton.IsEnabled = false;
            ClearButton.IsEnabled = false;
            progressbar1.Visibility = Visibility.Hidden;
            Parser = new MessageParser();
            BP = new BINParser();
        }

        

    }
}