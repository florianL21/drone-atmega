using System;
using System.Collections.Generic;
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
using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;
using System.Collections.ObjectModel;
using Microsoft.Win32;
using System.IO;
using System.Globalization;

namespace ControlCenter
{
    /// <summary>
    /// Interaktionslogik für LoggingGraph.xaml
    /// </summary>
    public partial class LoggingGraph : Window
    {
        public int test = 0;
        public bool IsOpen = false;

        public PlotModel MyModel = new PlotModel();
        public LinearAxis xAxis = new LinearAxis { Position = AxisPosition.Bottom, Minimum = -10, Maximum = 0, Title = "Seconds", Unit = "s", IsPanEnabled = false, IsZoomEnabled = false};
        public LinearAxis yAxis = new LinearAxis { Position = AxisPosition.Left, Minimum = 0, Maximum = 100, IsPanEnabled = false, IsZoomEnabled = false };
        //public LineSeries MyData = new LineSeries();
        DateTime ApplicationStartTime = DateTime.Now;
        List<string> lineDescriptions = new List<string>();
        public List<LineSeries> allLineSeries = new List<LineSeries>();
        double timeToDisplay = 10;
        double[,] xAxisMaxMins;
        int MaxUpdatesPerSecond = 0;
        DateTime[] timeOfLastUpdate;
        bool GraphPaused = false;
        string GraphDataSnapshot = "";

        public ObservableCollection<BoolStringClass> CheckboxList { get; set; }

        public class BoolStringClass
        {
            public string LineDescription { get; set; }
            public bool IsSelected { get; set; }
        }

        private double map(double x, double in_min, double in_max, double out_min, double out_max)
        {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

        public LoggingGraph()
        {
            InitializeComponent();
            Graph.Model = MyModel;
            MyModel.Axes.Add(xAxis);
            MyModel.Axes.Add(yAxis);
            IsOpen = true;

            CheckboxList = new ObservableCollection<BoolStringClass>();
            this.DataContext = this;
        }

        public void openInViewerMode(List<LineSeries> LineSeriesList, string[] lineDescription, double[,] AxisMaxMins, int maxUpdatesPerSecond)
        {
            xAxisMaxMins = AxisMaxMins;
            MaxUpdatesPerSecond = maxUpdatesPerSecond;
            timeOfLastUpdate = new DateTime[lineDescription.Length];
            DateTime temp = DateTime.Now;
            for (int i = 0; i < lineDescription.Length; i++)
            {
                timeOfLastUpdate[i] = temp;
                LineSeriesList[i].Smooth = true;
                LineSeriesList[i].Title = lineDescription[i];
                allLineSeries.Add(LineSeriesList[i]);
                MyModel.Series.Add(allLineSeries[i]);
                lineDescriptions.Add(lineDescription[i]);
                CheckboxList.Add(new BoolStringClass { IsSelected = true, LineDescription = lineDescription[i] });
            }
            xAxis.IsPanEnabled = true;
            xAxis.IsZoomEnabled = true;
            yAxis.IsPanEnabled = true;
            yAxis.IsZoomEnabled = true;
            MyModel.InvalidatePlot(true);
            TextBox_SecondsToShow.IsEnabled = false;
            Button_PauseResumeLogging.IsEnabled = false;
        }

        public void setDataLines(string[] lineDescription, double[,] AxisMaxMins, int maxUpdatesPerSecond)
        {
            xAxisMaxMins = AxisMaxMins;
            MaxUpdatesPerSecond = maxUpdatesPerSecond;
            timeOfLastUpdate = new DateTime[lineDescription.Length];
            DateTime temp = DateTime.Now;
            for (int i = 0; i < lineDescription.Length; i++)
            {
                timeOfLastUpdate[i] = temp;
                allLineSeries.Add(new LineSeries { Title = lineDescription[i], Smooth = true });
                MyModel.Series.Add(allLineSeries[i]);
                lineDescriptions.Add(lineDescription[i]);
                CheckboxList.Add(new BoolStringClass { IsSelected = true, LineDescription = lineDescription[i] });
            }
        }

        public void addDataPoint(double data, string lineDescription)
        {
            int lineIndex = lineDescriptions.IndexOf(lineDescription);
            if ((DateTime.Now - timeOfLastUpdate[lineIndex]).TotalMilliseconds >= MaxUpdatesPerSecond * 10)
            {
                timeOfLastUpdate[lineIndex] = DateTime.Now;
                double timeDiff = (DateTime.Now - ApplicationStartTime).TotalMilliseconds / 1000;
                LineSeries thisLineSeries = allLineSeries[lineIndex];
                thisLineSeries.Points.Add(new DataPoint(timeDiff, map(data, xAxisMaxMins[1, lineIndex], xAxisMaxMins[0, lineIndex], yAxis.Minimum, yAxis.Maximum)));

                xAxis.Minimum = timeDiff - timeToDisplay;
                xAxis.Maximum = timeDiff;
                if (thisLineSeries.Points.Count >= (timeToDisplay * MaxUpdatesPerSecond) * 2) //start deleting one cycle of data points when the next one is complete
                {
                    thisLineSeries.Points.RemoveRange(0, thisLineSeries.Points.Count - ((int)timeToDisplay * MaxUpdatesPerSecond));
                }
                if(GraphPaused == false)
                {
                    MyModel.InvalidatePlot(true);
                }
            } else
            {

            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            IsOpen = false;
        }

        private void TextBox_SecondsToShow_TextChanged(object sender, TextChangedEventArgs e)
        {
            try
            {
                timeToDisplay = double.Parse(TextBox_SecondsToShow.Text);
                xAxis.Minimum = -timeToDisplay;
            }catch(Exception error)
            {

            }
            if (GraphPaused == false)
            {
                MyModel.InvalidatePlot(true);
            }
        }

        private void CheckBox_Changed(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < CheckboxList.Count; i++)
            {
                LineSeries thisLineSeries = allLineSeries[lineDescriptions.IndexOf(CheckboxList.ElementAt(i).LineDescription)];
                thisLineSeries.IsVisible = CheckboxList.ElementAt(i).IsSelected;
            }
            if (GraphPaused == false)
            {
                MyModel.InvalidatePlot(true);
            }

        }

        string SerializeDataPlotWindow(string windowTitle, List<LineSeries> myData, List<string> LineDescriptions)
        {
            String temp = windowTitle + "\n";
            int LineCount = LineDescriptions.Count;
            temp += xAxis.Maximum.ToString(new CultureInfo("en-US")) + "," + xAxis.Minimum.ToString(new CultureInfo("en-US")) + "\n";
            temp += yAxis.Maximum.ToString(new CultureInfo("en-US")) + "," + yAxis.Minimum.ToString(new CultureInfo("en-US"));
            temp += "\t";
            for (int i = 0; i < LineCount; i++)
            {
                temp += "\n" + LineDescriptions[i];
            }
            temp += "\t";
            for (int i = 0; i < LineCount; i++)
            {
                temp += "\n" + xAxisMaxMins[0,i] + "," + xAxisMaxMins[1, i];
            }
            temp += "\t";
            for (int i = 0; i < LineCount; i++)
            {
                temp += "\t";
                foreach (DataPoint onePoint in myData[i].Points)
                {
                    temp += "\n" + onePoint.X.ToString(new CultureInfo("en-US")) + "," + onePoint.Y.ToString(new CultureInfo("en-US"));
                }
            }
            return temp;
        }

        private void Button_PauseResumeLogging_Click(object sender, RoutedEventArgs e)
        {
            if(GraphPaused == false)
            {
                ComboBox_DataLineEnable.IsEnabled = false;
                TextBox_SecondsToShow.IsEnabled = false;
                MenuItem_SaveGraph.IsEnabled = true;
                Button_PauseResumeLogging.Content = "Resume logging";
                GraphDataSnapshot = SerializeDataPlotWindow(MyModel.Title, allLineSeries, lineDescriptions);
                GraphPaused = true;
            }
            else
            {
                ComboBox_DataLineEnable.IsEnabled = true;
                TextBox_SecondsToShow.IsEnabled = true;
                Button_PauseResumeLogging.Content = "Pause logging";
                GraphPaused = false;
            }
        }

        private void MenuItem_SaveGraph_Click(object sender, RoutedEventArgs e)
        {
            SaveFileDialog saveFileDialog = new SaveFileDialog();
            saveFileDialog.Filter = "Graph Files (*.graph)|*.graph";
            saveFileDialog.AddExtension = true;
            if (saveFileDialog.ShowDialog() == true)
                File.WriteAllText(saveFileDialog.FileName, GraphDataSnapshot);
        }
    }
}
