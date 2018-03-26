using Microsoft.Win32;
using OxyPlot.Series;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Management;
using System.Collections.ObjectModel;

namespace ControlCenter
{
    /// <summary>
    /// Interaktionslogik für MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        SerialCom SerialPort;

        //System.Windows.Threading.DispatcherTimer dispatcherTestData = new System.Windows.Threading.DispatcherTimer();

        static TextBox StatusText;

        LoggingGraph MotorValueGraphWindow;
        LoggingGraph RCReceiverValueGraphWindow;
        LoggingGraph SensorValueGraphWindow;

        public EventLogWindow LogWindow;

        

        public MainWindow()
        {
            InitializeComponent();
            StatusText = StatusTextBox as TextBox;
            SerialPort = new SerialCom(this);
            /*
            dispatcherTestData.Tick += new EventHandler(sendTestData);
            dispatcherTestData.Interval = new TimeSpan(0, 0, 0, 0, 100);
            */
            

            using (var searcher = new ManagementObjectSearcher("SELECT * FROM Win32_PnPEntity WHERE Caption like '%(COM%'"))
            {
                var portnames = System.IO.Ports.SerialPort.GetPortNames();
                var ports = searcher.Get().Cast<ManagementBaseObject>().ToList().Select(p => p["Caption"].ToString());

                var portList = portnames.Select(n => n + " - " + ports.FirstOrDefault(s => s.Contains(n))).ToList();

                foreach (string s in portList)
                {
                    Combobox_PortNames.Items.Add(s);
                }
            }

            LogWindow = new EventLogWindow();
        }

        /*
        void sendTestData(object s, EventArgs e)
        {
            Random rand = new Random();
            if (MotorValueGraphWindow != null && MotorValueGraphWindow.IsOpen == true)
            {
                MotorValueGraphWindow.addDataPoint(rand.Next(0, 5200), "M0");
                MotorValueGraphWindow.addDataPoint(rand.Next(0, 5200), "M1");
                MotorValueGraphWindow.addDataPoint(rand.Next(0, 5200), "M2");
                MotorValueGraphWindow.addDataPoint(rand.Next(0, 5200), "M3");
            }
        }
        */
        /* Data Diplay Functions:
         */

        public void DisplayMotorData(int M0, int M1, int M2, int M3)
        {
            if(M0 == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_MotorValuesM0.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_MotorValuesM0.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_MotorValuesM0.Value = M0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_MotorValuesM0.Text = M0.ToString()));
                if (MotorValueGraphWindow != null && MotorValueGraphWindow.IsOpen == true)
                {
                    MotorValueGraphWindow.addDataPoint(M0, "M0");
                }
            }
            if (M1 == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_MotorValuesM1.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_MotorValuesM1.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_MotorValuesM1.Value = M1));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_MotorValuesM1.Text = M1.ToString()));
                if (MotorValueGraphWindow != null && MotorValueGraphWindow.IsOpen == true)
                {
                    MotorValueGraphWindow.addDataPoint(M1, "M1");
                }
            }
            if (M2 == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_MotorValuesM2.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_MotorValuesM2.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_MotorValuesM2.Value = M2));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_MotorValuesM2.Text = M2.ToString()));
                if (MotorValueGraphWindow != null && MotorValueGraphWindow.IsOpen == true)
                {
                    MotorValueGraphWindow.addDataPoint(M2, "M2");
                }
            }
            if (M3 == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_MotorValuesM3.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_MotorValuesM3.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_MotorValuesM3.Value = M3));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_MotorValuesM3.Text = M3.ToString()));
                if (MotorValueGraphWindow != null && MotorValueGraphWindow.IsOpen == true)
                {
                    MotorValueGraphWindow.addDataPoint(M3, "M3");
                }
            }
        }

        public void DisplayRCReaderData(int Throttle, int Roll, int Pitch, int Yaw, int Gear)
        {
            if (Throttle == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesThrottle.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesThrottle.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesThrottle.Value = Throttle));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesThrottle.Text = Throttle.ToString()));
                if (RCReceiverValueGraphWindow != null && RCReceiverValueGraphWindow.IsOpen == true)
                {
                    RCReceiverValueGraphWindow.addDataPoint(Throttle, "Throttle");
                }
            }
            if (Roll == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesRoll.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesRoll.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesRoll.Value = Roll));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesRoll.Text = Roll.ToString()));
                if (RCReceiverValueGraphWindow != null && RCReceiverValueGraphWindow.IsOpen == true)
                {
                    RCReceiverValueGraphWindow.addDataPoint(Roll, "Roll");
                }
            }
            if (Pitch == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesPitch.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesPitch.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesPitch.Value = Pitch));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesPitch.Text = Pitch.ToString()));
                if (RCReceiverValueGraphWindow != null && RCReceiverValueGraphWindow.IsOpen == true)
                {
                    RCReceiverValueGraphWindow.addDataPoint(Pitch, "Pitch");
                }
            }
            if (Yaw == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesYaw.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesYaw.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesYaw.Value = Yaw));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesYaw.Text = Yaw.ToString()));
                if (RCReceiverValueGraphWindow != null && RCReceiverValueGraphWindow.IsOpen == true)
                {
                    RCReceiverValueGraphWindow.addDataPoint(Yaw, "Yaw");
                }
            }
            if (Gear == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesGear.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesGear.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_RCReceiverValuesGear.Value = Gear));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_RCReceiverValuesGear.Text = Gear.ToString()));
                if (RCReceiverValueGraphWindow != null && RCReceiverValueGraphWindow.IsOpen == true)
                {
                    RCReceiverValueGraphWindow.addDataPoint(Gear, "Gear");
                }
            }
        }

        public void DisplaySensorData(int X, int Y, int Z)
        {
            if (X == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_SensorValuesRoll.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_SensorValuesRoll.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_SensorValuesRoll.Value = X));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_SensorValuesRoll.Text = X.ToString()));
                if (SensorValueGraphWindow != null && SensorValueGraphWindow.IsOpen == true)
                {
                    SensorValueGraphWindow.addDataPoint(X, "X");
                }
            }
            if (Y == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_SensorValuesPitch.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_SensorValuesPitch.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_SensorValuesPitch.Value = Y));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_SensorValuesPitch.Text = Y.ToString()));
                if (SensorValueGraphWindow != null && SensorValueGraphWindow.IsOpen == true)
                {
                    SensorValueGraphWindow.addDataPoint(Y, "Y");
                }
            }
            if (Z == -100000)
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_SensorValuesYaw.Value = 0));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_SensorValuesYaw.Text = "N/A"));
            }
            else
            {
                Dispatcher.BeginInvoke((Action)(() => ProgressBar_SensorValuesYaw.Value = Z));
                Dispatcher.BeginInvoke((Action)(() => TextBlock_SensorValuesYaw.Text = Z.ToString()));
                if (SensorValueGraphWindow != null && SensorValueGraphWindow.IsOpen == true)
                {
                    SensorValueGraphWindow.addDataPoint(Z, "Z");
                }
            }
        }

        public void DisplayRollPIDData(float kp, float ki, float kd)
        {
            Dispatcher.BeginInvoke((Action)(() => TextBox_Roll_P.Text = string.Format("{0:N4}", kp)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Roll_I.Text = string.Format("{0:N4}", ki)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Roll_D.Text = string.Format("{0:N4}", kd)));
            Dispatcher.BeginInvoke((Action)(() => Label_Roll_P_Arduino.Content = string.Format("{0:N4}", kp)));
            Dispatcher.BeginInvoke((Action)(() => Label_Roll_I_Arduino.Content = string.Format("{0:N4}", ki)));
            Dispatcher.BeginInvoke((Action)(() => Label_Roll_D_Arduino.Content = string.Format("{0:N4}", kd)));
        }

        public void DisplayPitchPIDData(float kp, float ki, float kd)
        {
            Dispatcher.BeginInvoke((Action)(() => TextBox_Pitch_P.Text = string.Format("{0:N4}", kp)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Pitch_I.Text = string.Format("{0:N4}", ki)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Pitch_D.Text = string.Format("{0:N4}", kd)));
            Dispatcher.BeginInvoke((Action)(() => Label_Pitch_P_Arduino.Content = string.Format("{0:N4}", kp)));
            Dispatcher.BeginInvoke((Action)(() => Label_Pitch_I_Arduino.Content = string.Format("{0:N4}", ki)));
            Dispatcher.BeginInvoke((Action)(() => Label_Pitch_D_Arduino.Content = string.Format("{0:N4}", kd)));
        }

        public void DisplayYawPIDData(float kp, float ki, float kd)
        {
            Dispatcher.BeginInvoke((Action)(() => TextBox_Yaw_P.Text = string.Format("{0:N4}", kp)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Yaw_I.Text = string.Format("{0:N4}", ki)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Yaw_D.Text = string.Format("{0:N4}", kd)));
            Dispatcher.BeginInvoke((Action)(() => Label_Yaw_P_Arduino.Content = string.Format("{0:N4}", kp)));
            Dispatcher.BeginInvoke((Action)(() => Label_Yaw_I_Arduino.Content = string.Format("{0:N4}", ki)));
            Dispatcher.BeginInvoke((Action)(() => Label_Yaw_D_Arduino.Content = string.Format("{0:N4}", kd)));
        }

        /* Logging Functions:
         */

        

        public void SetStatus(string statusText)
        {
            Dispatcher.BeginInvoke((Action)(() => StatusText.Text = "[" + DateTime.Now.ToString("h:mm:ttss") + "]: " + statusText));
        }

        /* Misc Functions:
         */
        public void RefreshPIDValues()
        {
            SerialPort.sendMessage_and_wait_for_ack(0x05, 'Y');
            SerialPort.sendMessage_and_wait_for_ack(0x05, 'P');
            SerialPort.sendMessage_and_wait_for_ack(0x05, 'R');
        }

        public void ResetGUI()
        {
            CheckBox_SensorValuesEnable.IsChecked = false;
            CheckBox_RCReceiverValuesEnable.IsChecked = false;
            CheckBox_MotorValuesEnable.IsChecked = false;
            DisplayMotorData(-100000, -100000, -100000, -100000);
            DisplayRCReaderData(-100000, -100000, -100000, -100000, -100000);
            DisplaySensorData(-100000, -100000, -100000);
            DisplayRollPIDData(0, 0, 0);
            DisplayPitchPIDData(0, 0, 0);
            DisplayYawPIDData(0, 0, 0);
        }

        void sendPIDValuesToArduino(byte PIDIdentifier, float kp, float ki, float kd)
        {
            byte[] buffer = new byte[16];
            byte[] Value = new byte[4];

            buffer[0] = PIDIdentifier;
            buffer[1] = (byte)'P';
            Value = BitConverter.GetBytes(kp);
            buffer[2] = Value[3];
            buffer[3] = Value[2];
            buffer[4] = Value[1];
            buffer[5] = Value[0];

            buffer[6] = (byte)'I';
            Value = BitConverter.GetBytes(ki);
            buffer[7] = Value[3];
            buffer[8] = Value[2];
            buffer[9] = Value[1];
            buffer[10] = Value[0];

            buffer[11] = (byte)'D';
            Value = BitConverter.GetBytes(kd);
            buffer[12] = Value[3];
            buffer[13] = Value[2];
            buffer[14] = Value[1];
            buffer[15] = Value[0];
            SerialPort.
            sendMessage_and_wait_for_ack(0x06, buffer);
        }

        /*Button Clicks:
         */

        void Connect_DisconnectButton_Click(object sender, RoutedEventArgs e)
        {
            SerialPort.Clear();
            if (SerialPort.IsOpen())
            {
                try
                {
                    SerialPort.sendMessage(0x01, 'O');
                    SerialPort.sendMessage(0x02, 'O');
                    SerialPort.sendMessage(0x03, 'O');
                    Connect_DisconnectButton.Content = "Connect";
                    SetStatus("Disconnected");
                    GroupBox_MotorValues.IsEnabled = false;
                    GroupBox_RCReceiverValues.IsEnabled = false;
                    GroupBox_SensorValues.IsEnabled = false;
                    GroupBox_PIDValues.IsEnabled = false;
                    Combobox_PortNames.IsEnabled = true;
                    Button_StartRCReceiverValuesGraph.IsEnabled = false;
                    Button_StartMotorValuesGraph.IsEnabled = false;
                    Button_StartSensorValuesGraph.IsEnabled = false;
                    Button_ResetArduino.IsEnabled = false;
                    CheckBox_ClearLogOnReset.IsEnabled = false;
                    if (SensorValueGraphWindow != null && SensorValueGraphWindow.IsOpen == true)
                        SensorValueGraphWindow.Close();
                    if (MotorValueGraphWindow != null && MotorValueGraphWindow.IsOpen == true)
                        MotorValueGraphWindow.Close();
                    if (RCReceiverValueGraphWindow != null && RCReceiverValueGraphWindow.IsOpen == true)
                        RCReceiverValueGraphWindow.Close();
                    ResetGUI();
                    SerialPort.Close();
                }
                catch (Exception Error)
                {
                    LogWindow.WriteToLog(EventLogWindow.LogTypes.EXCEPTION, "Connect_DisconnectButton_Click", "Exception Thrown: " + Error.ToString());
                    SetStatus("Error closing serial COM");
                }
            }
            else
            {
                
                if (Combobox_PortNames.SelectedIndex != -1)
                {
                    string com = Combobox_PortNames.SelectedValue.ToString();
                    try
                    {
                        SerialPort.setPort(com.Substring(0,com.IndexOf(' ')));
                        SerialPort.Open();
                        Connect_DisconnectButton.Content = "Disconnect";
                        SetStatus("Connected");
                        GroupBox_MotorValues.IsEnabled = true;
                        GroupBox_RCReceiverValues.IsEnabled = true;
                        GroupBox_SensorValues.IsEnabled = true;
                        GroupBox_PIDValues.IsEnabled = true;
                        Combobox_PortNames.IsEnabled = false;
                        Button_ResetArduino.IsEnabled = true;
                        CheckBox_ClearLogOnReset.IsEnabled = true;
                        RefreshPIDValues();
                    }
                    catch (Exception Error)
                    {
                        LogWindow.WriteToLog(EventLogWindow.LogTypes.EXCEPTION, "Connect_DisconnectButton_Click", "Exception Thrown: " + Error.ToString());
                        SetStatus("Error opening serial " + com.Substring(0, com.IndexOf(' ')));
                    }
                }
                else
                {
                    SetStatus("Please specify a port");
                }
            }
        }

        private void Button_RollPIDSave_Click(object sender, RoutedEventArgs e)
        {
            float kp, ki, kd;
            kp = float.Parse(TextBox_Roll_P.Text);
            ki = float.Parse(TextBox_Roll_I.Text);
            kd = float.Parse(TextBox_Roll_D.Text);

            /*if ((kp >= 32768 || ki >= 32768 || kd >= 32768) || (kp <= -32768 || ki <= -32768 || kd <= -32768))
            {
                SetStatus("Value has to be in between -327,67 and 327,67");
                return;
            }*/

            sendPIDValuesToArduino((byte)'R', kp, ki, kd);

            RefreshPIDValues();

            SetStatus("Roll PID Values Saved\tP: " + TextBox_Roll_P.Text + "\tI: " + TextBox_Roll_I.Text + "\tD: " + TextBox_Roll_D.Text);
        }

        private void Button_PitchPIDSave_Click(object sender, RoutedEventArgs e)
        {

            float kp, ki, kd;
            kp = float.Parse(TextBox_Pitch_P.Text);
            ki = float.Parse(TextBox_Pitch_I.Text);
            kd = float.Parse(TextBox_Pitch_D.Text);

            /*if ((kp >= 32768 || ki >= 32768 || kd >= 32768) || (kp <= -32768 || ki <= -32768 || kd <= -32768))
            {
                SetStatus("Value has to be in between -327,67 and 327,67");
                return;
            }*/

            sendPIDValuesToArduino((byte)'P', kp, ki, kd);

            RefreshPIDValues();

            SetStatus("Pitch PID Values Saved\tP: " + TextBox_Pitch_P.Text + "\tI: " + TextBox_Pitch_I.Text + "\tD: " + TextBox_Pitch_D.Text);
        }

        private void Button_YawPIDSave_Click(object sender, RoutedEventArgs e)
        {
            float kp, ki, kd;
            kp = float.Parse(TextBox_Yaw_P.Text);
            ki = float.Parse(TextBox_Yaw_I.Text);
            kd = float.Parse(TextBox_Yaw_D.Text);

            /*if ((kp >= 32768 || ki >= 32768 || kd >= 32768) || (kp <= -32768 || ki <= -32768 || kd <= -32768))
            {
                SetStatus("Value has to be in between -327,67 and 327,67");
                return;
            }*/

            sendPIDValuesToArduino((byte)'Y', kp, ki, kd);

            RefreshPIDValues();

            SetStatus("Yaw PID Values Saved\tP: " + TextBox_Yaw_P.Text + "\tI: " + TextBox_Yaw_I.Text + "\tD: " + TextBox_Yaw_D.Text);
        }

        private void Button_PIDSaveToFlash_Click(object sender, RoutedEventArgs e)
        {
            SerialPort.sendMessage(0x07, 'S');
        }

        private void Button_setSensorOffsets_Click(object sender, RoutedEventArgs e)
        {
            SerialPort.sendMessage(0x04, 'S');
        }

        private void Button_RefreshYawPID_Click(object sender, RoutedEventArgs e)
        {
            RefreshPIDValues();
        }

        private void Button_RefreshPitchPID_Click(object sender, RoutedEventArgs e)
        {
            RefreshPIDValues();
        }

        private void Button_RefreshRollPID_Click(object sender, RoutedEventArgs e)
        {
            RefreshPIDValues();
        }

        

        private void Button_StartSensorValuesGraph_Click(object sender, RoutedEventArgs e)
        {
            if(SensorValueGraphWindow == null || SensorValueGraphWindow.IsOpen == false)
            {
                SensorValueGraphWindow = new LoggingGraph();
                SensorValueGraphWindow.MyModel.Title = "Sensor Values";
                string[] dataLineNames = { "X", "Y", "Z" };
                double[,] maxMins = new double[,] { { 1024, 1024, 1024, 1024, 1 }, { -1024, -1024, -1024, -1024, -1024 } };
                SensorValueGraphWindow.setDataLines(dataLineNames, maxMins, 10); //updates set to a maximum of 10 times per second
                SensorValueGraphWindow.yAxis.Maximum = 1024;
                SensorValueGraphWindow.yAxis.Minimum = -1024;
                SensorValueGraphWindow.Show();
            }
            else
            {
                SensorValueGraphWindow.Activate();
            }
        }

        private void Button_StartRCReceiverValuesGraph_Click(object sender, RoutedEventArgs e)
        {
            if (RCReceiverValueGraphWindow == null || RCReceiverValueGraphWindow.IsOpen == false)
            {
                RCReceiverValueGraphWindow = new LoggingGraph();
                RCReceiverValueGraphWindow.MyModel.Title = "RC Receiver Values";
                string[] dataLineNames = { "Throttle", "Roll", "Pitch", "Yaw", "Gear" };
                double[,] maxMins = new double[,] { { 2200, 2200, 2200, 2200, 1 }, { 0, 0, 0, 0, 0 } };
                RCReceiverValueGraphWindow.setDataLines(dataLineNames, maxMins, 10); //updates set to a maximum of 10 times per second
                RCReceiverValueGraphWindow.yAxis.Maximum = 2200;
                RCReceiverValueGraphWindow.yAxis.Minimum = 0;
                RCReceiverValueGraphWindow.Show();
            }
            else
            {
                RCReceiverValueGraphWindow.Activate();
            }
        }
       
        private void Button_StartMotorValuesGraph_Click(object sender, RoutedEventArgs e)
        {
            if (MotorValueGraphWindow == null || MotorValueGraphWindow.IsOpen == false)
            {
                MotorValueGraphWindow = new LoggingGraph();
                MotorValueGraphWindow.MyModel.Title = "Motor Values";
                string[] dataLineNames = { "M0", "M1", "M2", "M3"};
                double[,] maxMins = new double[,] { { 5200, 5200, 5200, 5200 }, { 0, 0, 0, 0} };
                MotorValueGraphWindow.setDataLines(dataLineNames, maxMins, 10);  //updates set to a maximum of 10 times per second
                MotorValueGraphWindow.yAxis.Maximum = 5200;
                MotorValueGraphWindow.yAxis.Minimum = 0;
                MotorValueGraphWindow.Show();
                //dispatcherTestData.Start();
            }
            else
            {
                MotorValueGraphWindow.Activate();
            }
        }

        private void Button_ResetArduino_Click(object sender, RoutedEventArgs e)
        {
            SerialPort.sendMessage(0x04, 'R');
            if(CheckBox_ClearLogOnReset.IsChecked == true)
            {
                LogWindow.clearErrorLog();
            }
        }

        /*Checkboxes:
         */

        private void CheckBox_SensorValuesEnable_Checked(object sender, RoutedEventArgs e)
        {
            SerialPort.sendMessage_and_wait_for_ack(0x03, 'I');
            SetStatus("Sensor Value Logging Enabled: " + CheckBox_SensorValuesEnable.IsChecked.ToString());
            Button_StartSensorValuesGraph.IsEnabled = true;
        }

        private void CheckBox_SensorValuesEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            SerialPort.sendMessage_and_wait_for_ack(0x03, 'O');
            SetStatus("Sensor Value Logging Enabled: " + CheckBox_SensorValuesEnable.IsChecked.ToString());
            DisplaySensorData(-100000, -100000, -100000);
            Button_StartSensorValuesGraph.IsEnabled = false;
        }

        private void CheckBox_MotorValuesEnable_Checked(object sender, RoutedEventArgs e)
        {
            SerialPort.sendMessage_and_wait_for_ack(0x01, 'I');
            SetStatus("Motor Value Logging Enabled: " + CheckBox_MotorValuesEnable.IsChecked.ToString());
            Button_StartMotorValuesGraph.IsEnabled = true;
        }

        private void CheckBox_MotorValuesEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            SerialPort.sendMessage_and_wait_for_ack(0x01, 'O');
            SetStatus("Motor Value Logging Enabled: " + CheckBox_MotorValuesEnable.IsChecked.ToString());
            DisplayMotorData(-100000, -100000, -100000, -100000);
            Button_StartMotorValuesGraph.IsEnabled = false;
        }

        private void CheckBox_RCReceiverValuesEnable_Checked(object sender, RoutedEventArgs e)
        {
            SerialPort.sendMessage_and_wait_for_ack(0x02, 'I');
            SetStatus("RC Receiver Value Logging Enabled: " + CheckBox_RCReceiverValuesEnable.IsChecked.ToString());
            Button_StartRCReceiverValuesGraph.IsEnabled = true;
        }

        private void CheckBox_RCReceiverValuesEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            SerialPort.sendMessage_and_wait_for_ack(0x02, 'O');
            SetStatus("RC Receiver Value Logging Enabled: " + CheckBox_RCReceiverValuesEnable.IsChecked.ToString());
            DisplayRCReaderData(-100000, -100000, -100000, -100000, -100000);
            Button_StartRCReceiverValuesGraph.IsEnabled = false;
        }

        /* Special:
         */
        private void MenuItem_OpenGraph_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog openFileDialog = new OpenFileDialog();
            openFileDialog.Filter = "Graph Files(.graph)|*.graph";
            openFileDialog.Title = "Select a Cursor File";
            if (openFileDialog.ShowDialog() == true)
            {
                System.IO.StreamReader sr = new
                   System.IO.StreamReader(openFileDialog.FileName);
                DeserializeDataPlotWindow(sr.ReadToEnd());
                sr.Close();
            }
        }


        void DeserializeDataPlotWindow(string myData)
        {
            try
            {
                List<LineSeries> allLineSeries1 = new List<LineSeries>();
                LoggingGraph myViewer = new LoggingGraph();
                int startIndex = myData.IndexOf('\n');
                myViewer.MyModel.Title = myData.Substring(0, startIndex);
                myData = myData.Substring(startIndex + 1, myData.Length - (startIndex + 1));
                startIndex = myData.IndexOf(',');
                myViewer.xAxis.Maximum = double.Parse(myData.Substring(0, startIndex), new CultureInfo("en-US"));
                myData = myData.Substring(startIndex + 1, myData.Length - (startIndex + 1));
                startIndex = myData.IndexOf('\n');
                myViewer.xAxis.Minimum = double.Parse(myData.Substring(0, startIndex), new CultureInfo("en-US"));
                myData = myData.Substring(startIndex + 1, myData.Length - (startIndex + 1));
                startIndex = myData.IndexOf(',');
                myViewer.yAxis.Maximum = double.Parse(myData.Substring(0, startIndex), new CultureInfo("en-US"));
                myData = myData.Substring(startIndex + 1, myData.Length - (startIndex + 1));
                startIndex = myData.IndexOf('\t');
                myViewer.yAxis.Minimum = double.Parse(myData.Substring(0, startIndex), new CultureInfo("en-US"));
                myData = myData.Substring(startIndex + 1, myData.Length - (startIndex + 1));
                startIndex = myData.IndexOf('\t');
                string[] dataLineNames = myData.Substring(1, startIndex - 1).Split('\n');
                myData = myData.Substring(startIndex + 1, myData.Length - (startIndex + 1));
                startIndex = myData.IndexOf('\t');//there is a '\n' after this '\t' so it gets cut off
                string[] temp = myData.Substring(1, startIndex - 1).Split('\n');
                double[,] maxMins = new double[2, temp.Length];
                for (int i = 0; i < temp.Length; i++)
                {
                    string[] temp1 = temp[i].Split(',');
                    maxMins[0, i] = double.Parse(temp1[0], new CultureInfo("en-US"));
                    maxMins[1, i] = double.Parse(temp1[1], new CultureInfo("en-US"));
                }
                myData = myData.Substring(startIndex + 2, myData.Length - (startIndex + 2));
                temp = myData.Split('\t');
                for (int i = 0; i < temp.Length; i++)
                {
                    LineSeries thisLineSeries = new LineSeries();

                    temp[i] = temp[i].Substring(1, temp[i].Length - 1);
                    string[] points = temp[i].Split('\n');
                    for (int j = 0; j < points.Length; j++)
                    {
                        string[] onePoint = points[j].Split(',');
                        thisLineSeries.Points.Add(new OxyPlot.DataPoint(double.Parse(onePoint[0], new CultureInfo("en-US")), double.Parse(onePoint[1], new CultureInfo("en-US"))));
                    }
                    allLineSeries1.Add(thisLineSeries);
                }
                //myViewer.setDataLines(dataLineNames, maxMins, 10);  //updates set to a maximum of 10 times per second
                myViewer.openInViewerMode(allLineSeries1, dataLineNames, maxMins, 10);
                myViewer.Show();
            }
            catch(Exception error)
            {
                MessageBox.Show(error.ToString());
            }
        }

        private void StatusTextBox_TextChanged(object sender, TextChangedEventArgs e)
        {
            //StatusTextBox.ScrollToEnd();
        }

        private void MenuItem_OpenLog_Click(object sender, RoutedEventArgs e)
        {
            if (LogWindow != null && LogWindow.IsVisible == false)
            {
                LogWindow.Show();
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            SerialPort.sendMessage(0x01, 'O');
            SerialPort.sendMessage(0x02, 'O');
            SerialPort.sendMessage(0x03, 'O');
            Connect_DisconnectButton.Content = "Connect";
            SetStatus("Disconnected");
            ResetGUI();
            if (MotorValueGraphWindow != null && MotorValueGraphWindow.IsOpen == true)
            {
                MotorValueGraphWindow.Close();
            }
            if (SensorValueGraphWindow != null && SensorValueGraphWindow.IsOpen == true)
            {
                SensorValueGraphWindow.Close();
            }
            if (RCReceiverValueGraphWindow != null && RCReceiverValueGraphWindow.IsOpen == true)
            {
                RCReceiverValueGraphWindow.Close();
            }
            if (LogWindow != null)
            {
                LogWindow.Shutdown();
            }
            
            SerialPort.Close();
            SerialPort.Dispose();
        }
    }
}
