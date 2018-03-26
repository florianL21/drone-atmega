using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
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
using System.Windows.Shapes;

namespace ControlCenter
{
    /// <summary>
    /// Interaktionslogik für EventLogWindow.xaml
    /// </summary>
    public partial class EventLogWindow : Window
    {

        public ObservableCollection<ErrorLogEntry> ErrorLogList { get; set; }

        private bool isShuttingDown = false;

        public enum LogTypes
        {
            INFO,
            DEBUG,
            WARNING,
            ERROR,
            EXCEPTION
        }

        public class ErrorLogEntry
        {
            public string TimeInfo { get; set; }
            public string LogTypeInfo { get; set; }
            public string OriginatingFunctionInfo { get; set; }
            public string MessageInfo { get; set; }
            public LogTypes LogType { get; set; }
            public Brush LogEntryBackgoundColor { get; set; }
            public bool ShowThisEntry { get; set; }
        }

        public ObservableCollection<BoolStringClass> FilterCheckboxList { get; set; }

        public class BoolStringClass
        {
            public string LineDescription { get; set; }
            public bool IsSelected { get; set; }
        }

        public EventLogWindow()
        {
            InitializeComponent();
            ErrorLogList = new ObservableCollection<ErrorLogEntry>();
            FilterCheckboxList = new ObservableCollection<BoolStringClass>();
            foreach (LogTypes Type in Enum.GetValues(typeof(LogTypes)))
            {
                FilterCheckboxList.Add(new BoolStringClass { IsSelected = true, LineDescription = Type.ToString() });
            }
            this.DataContext = this;
        }

        public void clearErrorLog()
        {
            ErrorLogList.Clear();
        }

        private void Button_ClearLog_Click(object sender, RoutedEventArgs e)
        {
            ErrorLogList.Clear();
        }

        public void WriteToLog(LogTypes Type, string OriginatingFunction, string MessageDescription)
        {
            Brush bgColor;
            switch (Type)
            {
                case LogTypes.INFO:
                    bgColor = Brushes.White;
                    break;
                case LogTypes.DEBUG:
                    bgColor = Brushes.LightBlue;
                    break;
                case LogTypes.WARNING:
                    bgColor = Brushes.DarkOrange;
                    break;
                case LogTypes.ERROR:
                    bgColor = Brushes.OrangeRed;
                    Show();
                    break;
                case LogTypes.EXCEPTION:
                    bgColor = Brushes.Red;
                    Show();
                    break;
                default:
                    bgColor = Brushes.Yellow;
                    break;
            }
            ErrorLogList.Add(new ErrorLogEntry { ShowThisEntry = true, LogEntryBackgoundColor = bgColor, TimeInfo = DateTime.Now.ToString("HH:mm:ttss"), LogType = Type, LogTypeInfo = Type.ToString(), OriginatingFunctionInfo = OriginatingFunction, MessageInfo = MessageDescription });
        }

        public void Shutdown()
        {
            isShuttingDown = true;
            Close();
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if(isShuttingDown == false)
            {
                e.Cancel = true;  // cancels the window close    
                this.Hide();      // Programmatically hides the window
            }
        }

        private void CheckBox_Changed(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < FilterCheckboxList.Count; i++)
            {
                for (int j = 0; j < ErrorLogList.Count; j++)
                {
                    if (ErrorLogList.ElementAt(j).LogTypeInfo == FilterCheckboxList.ElementAt(i).LineDescription)
                    {
                        ErrorLogList.ElementAt(j).ShowThisEntry = FilterCheckboxList.ElementAt(i).IsSelected;
                    }
                }
            }
        }
    }
}
