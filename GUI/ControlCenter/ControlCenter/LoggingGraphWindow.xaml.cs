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
        DateTime lastTimeUpdated = DateTime.Now;
        List<string> lineDescriptions = new List<string>();
        List<LineSeries> allLineSeries = new List<LineSeries>();
        double timeToDisplay = 10;
        

        public LoggingGraph()
        {
            InitializeComponent();
            Graph.Model = MyModel;
            MyModel.Axes.Add(xAxis);
            MyModel.Axes.Add(yAxis);
            //MyModel.Series.Add(MyData);
            IsOpen = true;
        }

        public void setDataLines(string[] lineDescription, double[,] AxisMaxMins)
        {
            for (int i = 0; i < lineDescription.Length; i++)
            {
                allLineSeries.Add(new LineSeries { Title = lineDescription[i] });
                MyModel.Series.Add(allLineSeries[i]);
                lineDescriptions.Add(lineDescription[i]);
            }
        }

        public void addDataPoint(double data, string lineDescription)
        {
            double timeDiff = (DateTime.Now - lastTimeUpdated).TotalMilliseconds/1000;
            List<DataPoint> newPoints = new List<DataPoint>();
            LineSeries thisLineSeries = allLineSeries[lineDescriptions.IndexOf(lineDescription)];
            thisLineSeries.Points.Add(new DataPoint(timeDiff, data));
            xAxis.Minimum = timeDiff - timeToDisplay;
            xAxis.Maximum = timeDiff;
            if(thisLineSeries.Points.Count >= timeToDisplay*20) //est. 20 point per second 
            {
                thisLineSeries.Points.RemoveRange(0, thisLineSeries.Points.Count - ((int)timeToDisplay * 20));
            }
            MyModel.InvalidatePlot(true);
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
    }
}
