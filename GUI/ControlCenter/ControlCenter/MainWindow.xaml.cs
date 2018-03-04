using System;
using System.Collections.Generic;
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

namespace ControlCenter
{
    /// <summary>
    /// Interaktionslogik für MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public SerialPort mySerialPort;
        private Queue<byte> recievedData = new Queue<byte>();
        private Queue<byte[]> sendQueue = new Queue<byte[]>();
        bool readyToSend = true;

        static TextBox StatusText;

        public MainWindow()
        {
            InitializeComponent();
            StatusText = StatusTextBox as TextBox;

            mySerialPort = new SerialPort("COM5");
            mySerialPort.BaudRate = 115200;
            mySerialPort.Parity = Parity.None;
            mySerialPort.StopBits = StopBits.One;
            mySerialPort.DataBits = 8;
            mySerialPort.Handshake = Handshake.None;
            mySerialPort.RtsEnable = false;
            mySerialPort.ReceivedBytesThreshold = 1;
            mySerialPort.ReadTimeout = 10;
            mySerialPort.DataReceived += new SerialDataReceivedEventHandler(serialPort_DataReceived);
            StatusText.Text = "";
            
        }

        /*Send Message Functions:
         */

        void sendMessage(byte Type, char message)
        {
            byte[] msg = new byte[1];
            msg[0] = (byte)message;
            sendMessage(Type, msg);
        }

        void sendMessage(byte Type, byte message)
        {
            byte[] msg = new byte[1];
            msg[0] = message;
            sendMessage(Type, msg);
        }

        void sendMessage(byte Type, byte[] message)
        {
            byte[] sendMessage = new byte[message.Length + 4];
            sendMessage[0] = 0x02;
            sendMessage[1] = Type;
            sendMessage[2] = (byte)message.Length;
            for (int i = 0; i < message.Length; i++)
            {
                sendMessage[i + 3] = message[i];
            }
            sendMessage[message.Length + 3] = 0x03;
            if(readyToSend == true)
            {
                if (mySerialPort.IsOpen)
                    mySerialPort.Write(sendMessage, 0, sendMessage.Length);
                readyToSend = false;
            } else
            {
                sendQueue.Enqueue(sendMessage);
            }
        }

        public void Dispose()
        {
            if (mySerialPort != null)
            {
                mySerialPort.Dispose();
            }
        }


        /* Helper Functions:
         */

        string byteArrayToString(byte[] bArray)
        {
            char[] cArray = new char[bArray.Length];
            for(int i = 0; i < bArray.Length; i++)
            {
                cArray[i] = (char)bArray[i];
            }
            return new string(cArray);
        }

        /* Data receive processing:
         */

        bool IsTypeValid(int Type)
        {
            //                                                       ACK  ERROR
            int[] validTypes = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x64 };
            for (int i = 0; i < validTypes.Length; i++)
            {
                if (Type == validTypes[i])
                    return true;
            }
            return false;
        }

        void processSerialPortData(int offset)
        {
            try
            {
                int startCharIndex = 0, endCharIndex = 0;
                if (recievedData.Count - offset >= 3)
                {
                    startCharIndex = recievedData.ToList().IndexOf(0x02, offset);//search for beginning char
                    if (startCharIndex != -1)
                    {
                        if (IsTypeValid(recievedData.ToList()[startCharIndex + 1]))//chech if type matches
                        {
                            int MessageLength = recievedData.ToList()[startCharIndex + 2];
                            endCharIndex = startCharIndex + MessageLength + 3;
                            try
                            {
                                if (recievedData.ToList()[endCharIndex] == 0x03)
                                {
                                    var packet = Enumerable.Range(0, endCharIndex + offset + 1).Select(i => recievedData.Dequeue());
                                    byte[] message1 = packet.ToArray();
                                    
                                    string message = byteArrayToString(message1);
                                    processData(message[startCharIndex + 1], message.Substring(startCharIndex + 3, MessageLength));
                                    if (recievedData.Count > 3)
                                    {
                                        processSerialPortData(0);
                                    }
                                }
                                else
                                {
                                    processSerialPortData(startCharIndex + 1);
                                    LogError("[ATTEMPTING AUTOCORRECTION] End Char missmatch. Maybe out of sync?", "processSerialPortData");
                                }
                            }
                            catch
                            {
                                //LogError("[ATTEMPTING AUTOCORRECTION] Not enought data in input buffer for this message length. Maybe out of sync? ", "processSerialPortData");
                                processSerialPortData(startCharIndex + 1);
                            }
                        }
                        else
                        {
                            processSerialPortData(startCharIndex + 1);
                            LogError("[ATTEMPTING AUTOCORRECTION] Invalid Type. Maybe out of sync?", "processSerialPortData");
                        }
                    }
                }
            }
            catch (Exception error)
            {
                LogError("Exception thrown: " + error.ToString(), "processSerialPortData");
            }
        }

        void serialPort_DataReceived(object s, SerialDataReceivedEventArgs e)
        {
            try
            {
                byte[] data = new byte[mySerialPort.BytesToRead];
                mySerialPort.Read(data, 0, data.Length);
                data.ToList().ForEach(b => recievedData.Enqueue(b));
                // Determine if we have a "packet" in the queue
                processSerialPortData(0);
            }
            catch (Exception Error)
            {
                LogError("Exception thrown: " + Error.ToString(), "serialPort_DataReceived");
            }
        }

        void processData(int Type, string receivedText)
        {
            switch (Type)
            {
                case 0x00:
                    LogDebug(receivedText);
                    SetStatus(DateTime.Now.ToString("h:mm:ttss") + ": " + receivedText);
                    break;
                case 0x01:
                    int[] Motor_speeds = { 0, 0, 0, 0 };
                    //check message integetry:
                    if(receivedText[0] == '0' && receivedText[3] == '1' && receivedText[6] == '2' && receivedText[9] == '3')
                    {
                        Motor_speeds[0] = (receivedText[1] << 8) | receivedText[2];
                        Motor_speeds[1] = (receivedText[4] << 8) | receivedText[5];
                        Motor_speeds[2] = (receivedText[7] << 8) | receivedText[8];
                        Motor_speeds[3] = (receivedText[10] << 8) | receivedText[11];
                        DisplayMotorData(Motor_speeds[0], Motor_speeds[1], Motor_speeds[2], Motor_speeds[3]);
                    }
                    else
                    {
                        LogError("Motor Speed Value transmission has errors, discarding Data.", "processData");
                    }
                    break;
                case 0x02:
                    int Throttle, Roll, Pitch, Yaw, Gear;
                    //check message integetry:
                    if (receivedText[0] == 'T' && receivedText[3] == 'R' && receivedText[6] == 'P' && receivedText[9] == 'Y' && receivedText[12] == 'G')
                    {
                        Throttle = (receivedText[1] << 8) | receivedText[2];
                        Roll = (receivedText[4] << 8) | receivedText[5];
                        Pitch = (receivedText[7] << 8) | receivedText[8];
                        Yaw = (receivedText[10] << 8) | receivedText[11];
                        Gear = receivedText[13];
                        DisplayRCReaderData(Throttle, Roll, Pitch, Yaw, Gear);
                    }
                    else
                    {
                        LogError("RC Reader transmission has errors, discarding Data.", "processData");
                    }
                    break;
                case 0x03:
                    short X, Y, Z;
                    //check message integetry:
                    if (receivedText[0] == 'X' && receivedText[4] == 'Y' && receivedText[8] == 'Z')
                    {
                        X = Convert.ToInt16((receivedText[2] << 8) | receivedText[3]);
                        Y = Convert.ToInt16((receivedText[6] << 8) | receivedText[7]);
                        Z = Convert.ToInt16((receivedText[10] << 8) | receivedText[11]);

                        if (receivedText[1] == 1)
                            X *= -1;
                        if (receivedText[5] == 1)
                            Y *= -1;
                        if (receivedText[9] == 1)
                            Z *= -1;

                        DisplaySensorData(X, Y, Z);
                    }
                    else
                    {
                        LogError("Sensor transmission has errors, discarding Data.", "processData");
                    }
                    break;
                case 0x04:
                    
                    float kp, ki, kd;
                    if(receivedText[1] == 'P' && receivedText[5] == 'I' && receivedText[9] == 'D')
                    {
                        kp = ((receivedText[3] << 8) | receivedText[4]);
                        ki = ((receivedText[7] << 8) | receivedText[8]);
                        kd = ((receivedText[11] << 8) | receivedText[12]);

                        kp /= 100;
                        ki /= 100;
                        kd /= 100;

                        if (receivedText[2] == 1)
                            kp *= -1;
                        if (receivedText[6] == 1)
                            ki *= -1;
                        if (receivedText[10] == 1)
                            kd *= -1;

                        switch(receivedText[0])
                        {
                            case 'R':
                                SetStatus("Received Roll Data.");
                                DisplayRollPIDData(kp, ki, kd);
                                break;
                            case 'P':
                                SetStatus("Received Pitch Data.");
                                DisplayPitchPIDData(kp, ki, kd);
                                break;
                            case 'Y':
                                SetStatus("Received Yaw Data.");
                                DisplayYawPIDData(kp, ki, kd);
                                break;
                        }
                    }
                    else
                    {
                        LogError("PID transmission has errors, discarding Data.", "processData");
                    }
                    break;
                case 0x05:
                    if (receivedText[0] == 'R')
                    {
                        Dispatcher.BeginInvoke((Action)(() => ResetGUI()));
                        RefreshPIDValues();
                    }
                    break;
                case 0x06:
                    if (receivedText[0] == 'A')
                    {
                        if(readyToSend == false && sendQueue.Count >= 1)
                        {
                            byte[] sendMessage = sendQueue.Dequeue();
                            if (mySerialPort.IsOpen)
                                mySerialPort.Write(sendMessage, 0, sendMessage.Length);
                            if (sendQueue.Count == 0)
                                readyToSend = true;
                        }
                    } else if(receivedText[0] == 'N')
                    {
                        LogError("Got NACK", "processData");
                    }
                    break;
                case 0x64:
                    LogError(receivedText, "ArduinoBoard");
                    SetStatus("The Arduino reported an error");
                    break;
                default:
                    LogError("Unspecified Type received. Should not be possible!!!", "processData");
                    SetStatus("An error occured");
                    break;
            }
        }

        /* Data Diplay Functions:
         */

        void DisplayMotorData(int M0, int M1, int M2, int M3)
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
            }
        }

        void DisplayRCReaderData(int Throttle, int Roll, int Pitch, int Yaw, int Gear)
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
            }
        }

        void DisplaySensorData(int X, int Y, int Z)
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
            }
        }

        void DisplayRollPIDData(float kp, float ki, float kd)
        {
            Dispatcher.BeginInvoke((Action)(() => TextBox_Roll_P.Background = Brushes.LightGreen));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Roll_I.Background = Brushes.LightGreen));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Roll_D.Background = Brushes.LightGreen));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Roll_P.Text = string.Format("{0:N2}", kp)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Roll_I.Text = string.Format("{0:N2}", ki)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Roll_D.Text = string.Format("{0:N2}", kd)));
        }

        void DisplayPitchPIDData(float kp, float ki, float kd)
        {
            Dispatcher.BeginInvoke((Action)(() => TextBox_Pitch_P.Background = Brushes.LightGreen));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Pitch_I.Background = Brushes.LightGreen));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Pitch_D.Background = Brushes.LightGreen));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Pitch_P.Text = string.Format("{0:N2}", kp)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Pitch_I.Text = string.Format("{0:N2}", ki)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Pitch_D.Text = string.Format("{0:N2}", kd)));
        }

        void DisplayYawPIDData(float kp, float ki, float kd)
        {
            Dispatcher.BeginInvoke((Action)(() => TextBox_Yaw_P.Background = Brushes.LightGreen));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Yaw_I.Background = Brushes.LightGreen));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Yaw_D.Background = Brushes.LightGreen));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Yaw_P.Text = string.Format("{0:N2}", kp)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Yaw_I.Text = string.Format("{0:N2}", ki)));
            Dispatcher.BeginInvoke((Action)(() => TextBox_Yaw_D.Text = string.Format("{0:N2}", kd)));
        }

        /* Logging Functions:
         */

        void LogError(string ErrorMessage, string FunctionName)
        {
            Dispatcher.BeginInvoke((Action)(() => TextBox_ErrorLog.Text += "[" + DateTime.Now.ToString("h:mm:ttss") + "], [ERROR] at Function [" + FunctionName + "]: " + ErrorMessage + "\n"));
        }

        void LogDebug(string DebugMessage)
        {
            Dispatcher.BeginInvoke((Action)(() => TextBox_ErrorLog.Text += "[" + DateTime.Now.ToString("h:mm:ttss") + "], [DEBUG]: " + DebugMessage + "\n"));
        }

        void SetStatus(string statusText)
        {
            Dispatcher.BeginInvoke((Action)(() => StatusText.Text = "[" + DateTime.Now.ToString("h:mm:ttss") + "]: " + statusText));
        }

        /* Misc Functions:
         */
        void RefreshPIDValues()
        {
            sendMessage(0x05, 'Y');
            sendMessage(0x05, 'P');
            sendMessage(0x05, 'R');
        }

        void ResetGUI()
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

        /*Button Clicks:
         */

        private void Connect_DisconnectButton_Click(object sender, RoutedEventArgs e)
        {
            if (mySerialPort.IsOpen)
            {
                try
                {
                    sendMessage(0x01, 'O');
                    sendMessage(0x02, 'O');
                    sendMessage(0x03, 'O');
                    Connect_DisconnectButton.Content = "Connect";
                    SetStatus("Disconnected");
                    GroupBox_MotorValues.IsEnabled = false;
                    GroupBox_RCReceiverValues.IsEnabled = false;
                    GroupBox_SensorValues.IsEnabled = false;
                    GroupBox_PIDValues.IsEnabled = false;
                    PortNameTextbox.IsEnabled = true;
                    ResetGUI();
                    mySerialPort.Close();
                }
                catch (Exception Error)
                {
                    LogError("Exception Thrown: " + Error.ToString(), "Connect_DisconnectButton_Click");
                    SetStatus("Error opening serial COM");
                }
            }
            else
            {
                if (PortNameTextbox.Text != "")
                {
                    try
                    {
                        mySerialPort.PortName = PortNameTextbox.Text;
                        mySerialPort.Open();
                        Connect_DisconnectButton.Content = "Disconnect";
                        SetStatus("Connected");
                        GroupBox_MotorValues.IsEnabled = true;
                        GroupBox_RCReceiverValues.IsEnabled = true;
                        GroupBox_SensorValues.IsEnabled = true;
                        GroupBox_PIDValues.IsEnabled = true;
                        PortNameTextbox.IsEnabled = false;
                        RefreshPIDValues();
                    }
                    catch (Exception Error)
                    {
                        LogError("Exception Thrown: " + Error.ToString(), "Connect_DisconnectButton_Click");
                        SetStatus("Error closing serial COM");
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
            short Value;
            byte[] buffer = new byte[4];
            kp = float.Parse(TextBox_Roll_P.Text) * 100;
            ki = float.Parse(TextBox_Roll_I.Text) * 100;
            kd = float.Parse(TextBox_Roll_D.Text) * 100;

            if ((kp >= 32768 || ki >= 32768 || kd >= 32768) || (kp <= -32768 || ki <= -32768 || kd <= -32768))
            {
                SetStatus("Value has to be in between -327,67 and 327,67");
                return;
            }

            buffer[0] = 0x01;
            if (kp < 0)
            {
                kp *= -1;
                buffer[1] = 1;
            } else
            {
                buffer[1] = 0;
            }
            Value = Convert.ToInt16(kp);
            buffer[2] = (byte)((Value & 0xFF00) >> 8);
            buffer[3] = (byte)(Value & 0x00FF);
            sendMessage(0x06, buffer);

            buffer[0] = 0x02;
            if (ki < 0)
            {
                ki *= -1;
                buffer[1] = 1;
            } else
            {
                buffer[1] = 0;
            }
            Value = Convert.ToInt16(ki);
            buffer[2] = (byte)((Value & 0xFF00) >> 8);
            buffer[3] = (byte)(Value & 0x00FF);
            sendMessage(0x06, buffer);

            buffer[0] = 0x03;
            if (kd < 0)
            {
                kd *= -1;
                buffer[1] = 1;
            } else
            {
                buffer[1] = 0;
            }
            Value = Convert.ToInt16(kd);
            buffer[2] = (byte)((Value & 0xFF00) >> 8);
            buffer[3] = (byte)(Value & 0x00FF);
            sendMessage(0x06, buffer);

            RefreshPIDValues();


            SetStatus("Roll PID Values Saved\tP: " + TextBox_Roll_P.Text + "\tI: " + TextBox_Roll_I.Text + "\tD: " + TextBox_Roll_D.Text);
        }

        private void Button_PitchPIDSave_Click(object sender, RoutedEventArgs e)
        {
            float kp, ki, kd;
            short Value;
            byte[] buffer = new byte[4];
            kp = float.Parse(TextBox_Pitch_P.Text) * 100;
            ki = float.Parse(TextBox_Pitch_I.Text) * 100;
            kd = float.Parse(TextBox_Pitch_D.Text) * 100;

            if ((kp >= 32768 || ki >= 32768 || kd >= 32768) || (kp <= -32768 || ki <= -32768 || kd <= -32768))
            {
                SetStatus("Value has to be in between -327,67 and 327,67");
                return;
            }

            buffer[0] = 0x04;
            if (kp < 0)
            {
                kp *= -1;
                buffer[1] = 1;
            }
            else
            {
                buffer[1] = 0;
            }
            Value = Convert.ToInt16(kp);
            buffer[2] = (byte)((Value & 0xFF00) >> 8);
            buffer[3] = (byte)(Value & 0x00FF);
            sendMessage(0x06, buffer);

            buffer[0] = 0x05;
            if (ki < 0)
            {
                ki *= -1;
                buffer[1] = 1;
            }
            else
            {
                buffer[1] = 0;
            }
            Value = Convert.ToInt16(ki);
            buffer[2] = (byte)((Value & 0xFF00) >> 8);
            buffer[3] = (byte)(Value & 0x00FF);
            sendMessage(0x06, buffer);

            buffer[0] = 0x06;
            if (kd < 0)
            {
                kd *= -1;
                buffer[1] = 1;
            }
            else
            {
                buffer[1] = 0;
            }
            Value = Convert.ToInt16(kd);
            buffer[2] = (byte)((Value & 0xFF00) >> 8);
            buffer[3] = (byte)(Value & 0x00FF);
            sendMessage(0x06, buffer);

            RefreshPIDValues();

            SetStatus("Pitch PID Values Saved\tP: " + TextBox_Pitch_P.Text + "\tI: " + TextBox_Pitch_I.Text + "\tD: " + TextBox_Pitch_D.Text);
        }

        private void Button_YawPIDSave_Click(object sender, RoutedEventArgs e)
        {
            float kp, ki, kd;
            short Value;
            byte[] buffer = new byte[4];
            kp = float.Parse(TextBox_Yaw_P.Text) * 100;
            ki = float.Parse(TextBox_Yaw_I.Text) * 100;
            kd = float.Parse(TextBox_Yaw_D.Text) * 100;

            if ((kp >= 32768 || ki >= 32768 || kd >= 32768) || (kp <= -32768 || ki <= -32768 || kd <= -32768))
            {
                SetStatus("Value has to be in between -327,67 and 327,67");
                return;
            }

            buffer[0] = 0x07;
            if (kp < 0)
            {
                kp *= -1;
                buffer[1] = 1;
            }
            else
            {
                buffer[1] = 0;
            }
            Value = Convert.ToInt16(kp);
            buffer[2] = (byte)((Value & 0xFF00) >> 8);
            buffer[3] = (byte)(Value & 0x00FF);
            sendMessage(0x06, buffer);

            buffer[0] = 0x08;
            if (ki < 0)
            {
                ki *= -1;
                buffer[1] = 1;
            }
            else
            {
                buffer[1] = 0;
            }
            Value = Convert.ToInt16(ki);
            buffer[2] = (byte)((Value & 0xFF00) >> 8);
            buffer[3] = (byte)(Value & 0x00FF);
            sendMessage(0x06, buffer);

            buffer[0] = 0x09;
            if (kd < 0)
            {
                kd *= -1;
                buffer[1] = 1;
            }
            else
            {
                buffer[1] = 0;
            }
            Value = Convert.ToInt16(kd);
            buffer[2] = (byte)((Value & 0xFF00) >> 8);
            buffer[3] = (byte)(Value & 0x00FF);
            sendMessage(0x06, buffer);

            RefreshPIDValues();

            SetStatus("Yaw PID Values Saved\tP: " + TextBox_Yaw_P.Text + "\tI: " + TextBox_Yaw_I.Text + "\tD: " + TextBox_Yaw_D.Text);
        }


        private void Button_setSensorOffsets_Click(object sender, RoutedEventArgs e)
        {
            sendMessage(0x04, 'S');
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

        /*Checkboxes:
         */

        private void CheckBox_SensorValuesEnable_Checked(object sender, RoutedEventArgs e)
        {
            sendMessage(0x03, 'I');
            SetStatus("Sensor Value Logging Enabled: " + CheckBox_SensorValuesEnable.IsChecked.ToString());
        }

        private void CheckBox_SensorValuesEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            sendMessage(0x03, 'O');
            SetStatus("Sensor Value Logging Enabled: " + CheckBox_SensorValuesEnable.IsChecked.ToString());
            DisplaySensorData(-100000, -100000, -100000);
        }

        private void CheckBox_MotorValuesEnable_Checked(object sender, RoutedEventArgs e)
        {
            sendMessage(0x01, 'I');
            SetStatus("Motor Value Logging Enabled: " + CheckBox_MotorValuesEnable.IsChecked.ToString());
        }

        private void CheckBox_MotorValuesEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            sendMessage(0x01, 'O');
            SetStatus("Motor Value Logging Enabled: " + CheckBox_MotorValuesEnable.IsChecked.ToString());
            DisplayMotorData(-100000, -100000, -100000, -100000);
        }

        private void CheckBox_RCReceiverValuesEnable_Checked(object sender, RoutedEventArgs e)
        {
            sendMessage(0x02, 'I');
            SetStatus("RC Receiver Value Logging Enabled: " + CheckBox_RCReceiverValuesEnable.IsChecked.ToString());
        }

        private void CheckBox_RCReceiverValuesEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            sendMessage(0x02, 'O');
            SetStatus("RC Receiver Value Logging Enabled: " + CheckBox_RCReceiverValuesEnable.IsChecked.ToString());
            DisplayRCReaderData(-100000, -100000, -100000, -100000, -100000);
        }

        /*Textbox changes:
         */

        private void TextBox_Pitch_P_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox_Pitch_P.Background = Brushes.OrangeRed;
        }

        private void TextBox_Pitch_I_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox_Pitch_I.Background = Brushes.OrangeRed;
        }

        private void TextBox_Pitch_D_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox_Pitch_D.Background = Brushes.OrangeRed;
        }

        private void TextBox_Roll_P_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox_Roll_P.Background = Brushes.OrangeRed;
        }

        private void TextBox_Roll_I_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox_Roll_I.Background = Brushes.OrangeRed;
        }

        private void TextBox_Roll_D_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox_Roll_D.Background = Brushes.OrangeRed;
        }

        private void TextBox_Yaw_P_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox_Yaw_P.Background = Brushes.OrangeRed;
        }

        private void TextBox_Yaw_I_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox_Yaw_I.Background = Brushes.OrangeRed;
        }

        private void TextBox_Yaw_D_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox_Yaw_D.Background = Brushes.OrangeRed;
        }

        /* Special:
         */

        private void TextBox_ErrorLog_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (TabControl_MainModes.SelectedIndex == 1)
            {
                TextBox_ErrorLog.ScrollToEnd();
            }
        }
        private void StatusTextBox_TextChanged(object sender, TextChangedEventArgs e)
        {
            //StatusTextBox.ScrollToEnd();
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            sendMessage(0x01, 'O');
            sendMessage(0x02, 'O');
            sendMessage(0x03, 'O');
            Connect_DisconnectButton.Content = "Connect";
            SetStatus("Disconnected");
            ResetGUI();
            mySerialPort.Close();
            Dispose();
        }

        private void TabControl_MainModes_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if(TabControl_MainModes.SelectedIndex == 1)
            {
                TextBox_ErrorLog.ScrollToEnd();
            }
        }
    }
}
