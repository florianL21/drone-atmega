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
        List<LineSeries> allLineSeries = new List<LineSeries>();
        double timeToDisplay = 10;
        double[,] xAxisMaxMins;
        int MaxUpdatesPerSecond = 0;
        DateTime[] timeOfLastUpdate;

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
                MyModel.InvalidatePlot(true);
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
             MyModel.InvalidatePlot(true);
        }

        private void CheckBox_Changed(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < CheckboxList.Count; i++)
            {
                LineSeries thisLineSeries = allLineSeries[lineDescriptions.IndexOf(CheckboxList.ElementAt(i).LineDescription)];
                thisLineSeries.IsVisible = CheckboxList.ElementAt(i).IsSelected;
            }
            MyModel.InvalidatePlot(true);

        }
    }
}
