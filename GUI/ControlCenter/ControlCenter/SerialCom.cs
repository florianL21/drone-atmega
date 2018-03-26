using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlCenter
{
    class SerialCom
    {
        public SerialPort mySerialPort;
        private bool readyToSend = true;

        private Queue<byte> recievedData = new Queue<byte>();
        private Queue<byte[]> sendQueue = new Queue<byte[]>();

        private System.Windows.Threading.DispatcherTimer dispatcherACKTimeout = new System.Windows.Threading.DispatcherTimer();

        private MainWindow myMainWindow;

        public SerialCom(MainWindow MyMainWindow)
        {
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
            //StatusText.Text = "";


            dispatcherACKTimeout.Tick += new EventHandler(ACK_timeout);
            dispatcherACKTimeout.Interval = new TimeSpan(0, 0, 1);

            myMainWindow = MyMainWindow;
        }

        public bool IsOpen()
        {
            return mySerialPort.IsOpen;
        }

        public void Open()
        {
            mySerialPort.Open();
        }

        public void Close()
        {
            mySerialPort.Close();
        }

        public void Clear()
        {
            sendQueue.Clear();
            recievedData.Clear();
        }

        public void setPort(string PortName)
        {
            mySerialPort.PortName = PortName;
        }

        /*Send Message Functions:
         */
        private void ACK_timeout(object s, EventArgs e)
        {
            dispatcherACKTimeout.Stop();
            myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.INFO, "ACK_timeout", "ACK Timeout");
            if (sendQueue.Count == 0)
                readyToSend = true;
            else
            {
                if (mySerialPort.IsOpen)
                {
                    byte[] sendMessage = sendQueue.Dequeue();
                    mySerialPort.Write(sendMessage, 0, sendMessage.Length);
                    dispatcherACKTimeout.Start();
                }
            }
        }

        public void sendMessage(byte Type, char message)
        {
            byte[] msg = new byte[1];
            msg[0] = (byte)message;
            sendMessage(Type, msg);
        }

        public void sendMessage(byte Type, byte message)
        {
            byte[] msg = new byte[1];
            msg[0] = message;
            sendMessage(Type, msg);
        }

        public void sendMessage(byte Type, byte[] message)
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
            if (mySerialPort.IsOpen)
                mySerialPort.Write(sendMessage, 0, sendMessage.Length);
        }

        public void sendMessage_and_wait_for_ack(byte Type, char message)
        {
            byte[] msg = new byte[1];
            msg[0] = (byte)message;
            sendMessage_and_wait_for_ack(Type, msg);
        }

        public void sendMessage_and_wait_for_ack(byte Type, byte[] message)
        {
            if (mySerialPort.IsOpen)
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

                if (readyToSend == true)
                {
                    mySerialPort.Write(sendMessage, 0, sendMessage.Length);
                    dispatcherACKTimeout.Start();
                    readyToSend = false;
                }
                else
                {
                    sendQueue.Enqueue(sendMessage);
                }
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

        private string byteArrayToString(byte[] bArray)
        {
            char[] cArray = new char[bArray.Length];
            for (int i = 0; i < bArray.Length; i++)
            {
                cArray[i] = (char)bArray[i];
            }
            return new string(cArray);
        }

        private byte[] charArrayToByteArray(char[] Array)
        {
            byte[] bArray = new byte[Array.Length];
            for (int i = 0; i < Array.Length; i++)
            {
                bArray[i] = (byte)Array[i];
            }
            return bArray;
        }

        /* Data receive processing:
         */

        private bool IsTypeValid(int Type)
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

        private void processSerialPortData(int offset)
        {
            try
            {
                int startCharIndex = 0, endCharIndex = 0;
                if (recievedData.Count - offset >= 3)
                {
                    startCharIndex = recievedData.ToList().IndexOf(0x02, offset);//search for beginning char
                    if (startCharIndex != -1 && startCharIndex + 3 <= recievedData.Count)
                    {
                        if (IsTypeValid(recievedData.ToList()[startCharIndex + 1]))//check if type matches
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
                                    myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.INFO, "processSerialPortData", "[ATTEMPTING AUTOCORRECTION] End Char missmatch. Maybe out of sync?");
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
                            myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.INFO, "processSerialPortData", "[ATTEMPTING AUTOCORRECTION] Invalid Type. Maybe out of sync?");
                        }
                    }
                }
            }
            catch (Exception error)
            {
                myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.EXCEPTION, "processSerialPortData", "Exception thrown: " + error.ToString());
            }
        }

        private void serialPort_DataReceived(object s, SerialDataReceivedEventArgs e)
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
                myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.EXCEPTION, "serialPort_DataReceived", "Exception thrown: " + Error.ToString());
            }
        }

        private void processData(int Type, string receivedText)
        {
            switch (Type)
            {
                case 0x00:
                    myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.DEBUG, "Arduino", receivedText);
                    myMainWindow.SetStatus(DateTime.Now.ToString("h:mm:ttss") + ": " + receivedText);
                    break;
                case 0x01:
                    int[] Motor_speeds = { 0, 0, 0, 0 };
                    //check message integetry:
                    if (receivedText[0] == '0' && receivedText[3] == '1' && receivedText[6] == '2' && receivedText[9] == '3')
                    {
                        Motor_speeds[0] = (receivedText[1] << 8) | receivedText[2];
                        Motor_speeds[1] = (receivedText[4] << 8) | receivedText[5];
                        Motor_speeds[2] = (receivedText[7] << 8) | receivedText[8];
                        Motor_speeds[3] = (receivedText[10] << 8) | receivedText[11];
                        myMainWindow.DisplayMotorData(Motor_speeds[0], Motor_speeds[1], Motor_speeds[2], Motor_speeds[3]);
                    }
                    else
                    {
                        myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.WARNING, "processData", "Motor Speed Value transmission has errors, discarding Data.");
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
                        myMainWindow.DisplayRCReaderData(Throttle, Roll, Pitch, Yaw, Gear);
                    }
                    else
                    {
                        myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.WARNING, "processData", "RC Reader transmission has errors, discarding Data.");
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

                        myMainWindow.DisplaySensorData(X, Y, Z);
                    }
                    else
                    {
                        myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.WARNING, "processData", "Sensor transmission has errors, discarding Data.");
                    }
                    break;
                case 0x04:
                    float kp, ki, kd;
                    if (receivedText[1] == 'P' && receivedText[6] == 'I' && receivedText[11] == 'D')
                    {
                        byte[] array = new byte[4];
                        array[0] = (byte)receivedText[5];
                        array[1] = (byte)receivedText[4];
                        array[2] = (byte)receivedText[3];
                        array[3] = (byte)receivedText[2];
                        kp = BitConverter.ToSingle(array, 0);
                        array[0] = (byte)receivedText[10];
                        array[1] = (byte)receivedText[9];
                        array[2] = (byte)receivedText[8];
                        array[3] = (byte)receivedText[7];
                        ki = BitConverter.ToSingle(array, 0);
                        array[0] = (byte)receivedText[15];
                        array[1] = (byte)receivedText[14];
                        array[2] = (byte)receivedText[13];
                        array[3] = (byte)receivedText[12];
                        kd = BitConverter.ToSingle(array, 0);

                        switch (receivedText[0])
                        {
                            case 'R':
                                myMainWindow.SetStatus("Received Roll Data.");
                                myMainWindow.DisplayRollPIDData(kp, ki, kd);
                                break;
                            case 'P':
                                myMainWindow.SetStatus("Received Pitch Data.");
                                myMainWindow.DisplayPitchPIDData(kp, ki, kd);
                                break;
                            case 'Y':
                                myMainWindow.SetStatus("Received Yaw Data.");
                                myMainWindow.DisplayYawPIDData(kp, ki, kd);
                                break;
                        }
                    }
                    else
                    {
                        myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.WARNING, "processData", "PID transmission has errors, discarding Data.");
                    }
                    break;
                case 0x05:
                    if (receivedText[0] == 'R')
                    {
                        myMainWindow.Dispatcher.BeginInvoke((Action)(() => myMainWindow.ResetGUI()));
                        myMainWindow.RefreshPIDValues();
                    }
                    break;
                case 0x06:
                    if (receivedText[0] == 'A')
                    {
                        dispatcherACKTimeout.Stop();
                        if (sendQueue.Count == 0)
                        {
                            readyToSend = true;
                        }
                        else
                        {
                            byte[] sendMessage = sendQueue.Dequeue();
                            mySerialPort.Write(sendMessage, 0, sendMessage.Length);
                            dispatcherACKTimeout.Start();
                            if (sendQueue.Count == 0)
                            {
                                readyToSend = true;
                            }
                        }
                    }
                    break;
                case 0x64:
                    myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.ERROR, "ARDUINO", receivedText);
                    myMainWindow.SetStatus("The Arduino reported an error");
                    break;
                default:
                    myMainWindow.LogWindow.WriteToLog(EventLogWindow.LogTypes.ERROR, "processData", "Unspecified Type received. Should not be possible!!!");
                    myMainWindow.SetStatus("An error occured");
                    break;
            }
        }
    }
}
