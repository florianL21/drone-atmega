using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
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
    /// 

    public class ErrorLogEntry : INotifyPropertyChanged
    {
        private string _TimeInfo;
        private EventLogWindow.LogTypes _LogType;
        private string _LogTypeInfo;
        private string _OriginatingFunctionInfo;
        private string _MessageInfo;
        private Brush _LogEntryBackgoundColor;

        public string TimeInfo
        {
            get { return this._TimeInfo; }
            set
            {
                if (this._TimeInfo != value)
                {
                    this._TimeInfo = value;
                    this.NotifyPropertyChanged("TimeInfo");
                }
            }
        }
        public EventLogWindow.LogTypes LogType
        {
            get { return this._LogType; }
            set
            {
                if (this._LogType != value)
                {
                    this._LogType = value;
                    this.NotifyPropertyChanged("LogType");
                }
            }
        }
        public string LogTypeInfo
        {
            get { return this._LogTypeInfo; }
            set
            {
                if (this._LogTypeInfo != value)
                {
                    this._LogTypeInfo = value;
                    this.NotifyPropertyChanged("LogTypeInfo");
                }
            }
        }
        public string OriginatingFunctionInfo
        {
            get { return this._OriginatingFunctionInfo; }
            set
            {
                if (this._OriginatingFunctionInfo != value)
                {
                    this._OriginatingFunctionInfo = value;
                    this.NotifyPropertyChanged("OriginatingFunctionInfo");
                }
            }
        }
        public string MessageInfo
        {
            get { return this._MessageInfo; }
            set
            {
                if (this._MessageInfo != value)
                {
                    this._MessageInfo = value;
                    this.NotifyPropertyChanged("MessageInfo");
                }
            }
        }
        public Brush LogEntryBackgoundColor
        {
            get { return this._LogEntryBackgoundColor; }
            set
            {
                if (this._LogEntryBackgoundColor != value)
                {
                    this._LogEntryBackgoundColor = value;
                    this.NotifyPropertyChanged("LogEntryBackgoundColor");
                }
            }
        }


        private string name;
        public string Name
        {
            get { return this.name; }
            set
            {
                if (this.name != value)
                {
                    this.name = value;
                    this.NotifyPropertyChanged("Name");
                }
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        public void NotifyPropertyChanged(string propName)
        {
            if (this.PropertyChanged != null)
                this.PropertyChanged(this, new PropertyChangedEventArgs(propName));
        }
    }

    public partial class EventLogWindow : Window
    {
        public enum LogTypes
        {
            INFO,
            DEBUG,
            WARNING,
            ERROR,
            EXCEPTION
        }

        public ObservableCollection<ErrorLogEntry> ErrorLogList
        {
            get
            {
                return (ObservableCollection<ErrorLogEntry>)
                GetValue(ErrorLogListProperty);
            }
            set
            {
                SetValue(ErrorLogListProperty, value);
            }
        }

        public static readonly DependencyProperty ErrorLogListProperty = DependencyProperty.Register("LogList", typeof(ObservableCollection<ErrorLogEntry>), typeof(EventLogWindow), new PropertyMetadata(new ObservableCollection<ErrorLogEntry>()));

        private bool isShuttingDown = false;


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
                    if (!IsVisible)
                        Show();
                    break;
                case LogTypes.EXCEPTION:
                    bgColor = Brushes.Red;
                    if(!IsVisible)
                        Show();
                    break;
                default:
                    bgColor = Brushes.Yellow;
                    break;
            }
            //ErrorLogEntry newEntry = new ErrorLogEntry { LogEntryBackgoundColor = bgColor, TimeInfo = DateTime.Now.ToString("HH:mm:ttss"), LogType = Type, LogTypeInfo = Type.ToString(), OriginatingFunctionInfo = OriginatingFunction, MessageInfo = MessageDescription };
            //LogEventTable.Items.Add(new ErrorLogEntry { LogEntryBackgoundColor = bgColor, TimeInfo = DateTime.Now.ToString("HH:mm:ttss"), LogType = Type, OriginatingFunctionInfo = OriginatingFunction, MessageInfo = MessageDescription });
            //EventLogTable.RowBackground = bgColor;
            //ErrorLogList.Add(newEntry);
            Dispatcher.BeginInvoke((Action)(() => ErrorLogList.Add(new ErrorLogEntry { LogEntryBackgoundColor = bgColor, TimeInfo = DateTime.Now.ToString("HH:mm:ttss"), LogType = Type, LogTypeInfo = Type.ToString(), OriginatingFunctionInfo = OriginatingFunction, MessageInfo = MessageDescription })));
            
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
            ICollectionView cv = CollectionViewSource.GetDefaultView(EventLogTable.ItemsSource);
            cv.Filter = o =>
            {
                ErrorLogEntry p = o as ErrorLogEntry;
                for (int i = 0; i < FilterCheckboxList.Count; i++)
                {
                    if (p.LogType.ToString() == FilterCheckboxList.ElementAt(i).LineDescription.ToString())
                        return FilterCheckboxList.ElementAt(i).IsSelected == true;
                }
                return false;
            };
        }

        private void EventLogTable_DataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            //EventLogTable.ScrollIntoView(EventLogTable.Items.GetItemAt(EventLogTable.Items.Count - 1));
        }
    }
}
