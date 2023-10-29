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
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace Mission_Control
{

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        MissionControl control = new MissionControl();

        public MainWindow()
        {
            InitializeComponent();
        }

        private void SerialListLoaded(object sender, RoutedEventArgs e)
        {
            var ports = MissionControl.SerialPorts();
            seriaPortList.ItemsSource = ports;
            control.SetControls(accel_x, 
                                accel_y, 
                                accel_z, 
                                rads_x, 
                                rads_y, 
                                rads_z, 
                                ready_rb, 
                                ready_rb, 
                                flight_rb, 
                                deploy_rb, 
                                landed_rb, 
                                log_screen, 
                                Dispatcher);
        }

        private void CalibrateClick(object sender, RoutedEventArgs e)
        {
            control.SendCalibrate();
        }

        private void StartDataStreamClick(object sender, RoutedEventArgs e)
        {
            control.SendStartDataStream();
        }

        private void StopDataStreamClick(object sender, RoutedEventArgs e)
        {
            control.SendStopDataStream();
        }

        private void StartMissionClick(object sender, RoutedEventArgs e)
        {
            control.SendStartMission();
        }

        private void StopMissionClick(object sender, RoutedEventArgs e)
        {
            control.SendStopMission();
        }

        private void PortSelected(object sender, RoutedEventArgs e)
        {
            control.OpenPort(seriaPortList.SelectedIndex);
        }
    }
}
