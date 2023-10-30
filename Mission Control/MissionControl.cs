using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Threading;
using System.Windows.Controls;
using System.Windows.Threading;

namespace Mission_Control
{
    public class MissionControl
    {
        static MissionControl this_;
        SerialPort serial_port;
        bool run = true;
        Label accel_x;
        Label accel_y;
        Label accel_z;
        Label rads_x;
        Label rads_y;
        Label rads_z;
        RadioButton ready_rb1;
        RadioButton ready_rb2;
        RadioButton flight_rb;
        RadioButton deploy_rb;
        RadioButton landed_rb;
        TextBlock log_screen;
        Dispatcher dispatcher;
        string[] serial_port_names;

        enum MessageId
        {
            SENSOR_DATA = 1,
            LOG_DATA,
        }

        public MissionControl()
        {
            this_ = this;
            ThreadStart thread_delegate = new ThreadStart(MissionControl.SerialReader);
            Thread serial_task = new Thread(thread_delegate);
            serial_task.Start();
        }

        public static string[] SerialPorts()
        {
            this_.serial_port_names = SerialPort.GetPortNames();
            return this_.serial_port_names;
        }

        public static void SerialReader()
        {
            while (this_.serial_port == null && 
                   this_.run)
            {
                Thread.Sleep(1000);
            }

            while (this_.serial_port.IsOpen == false)
            {
                Thread.Sleep(1000);
            }

            while (this_.run)
            {
                int length = this_.serial_port.ReadByte();
                int message_id = this_.serial_port.ReadByte();
                var message = new byte[length];
                this_.serial_port.Read(message, 0, length);
                this_.ProcessMessage(message_id, message);
            }
        }

        internal void SendStartDataStream()
        {
            if (serial_port != null && serial_port.IsOpen)
            {
                byte[] msg = new byte[4];
                msg[0] = 1; // Length
                msg[1] = 6; // Start Data Stream ID
                serial_port.Write(msg, 0, msg.Length);
            }
        }

        internal void SendStopDataStream()
        {
            if (serial_port != null && serial_port.IsOpen)
            {
                byte[] msg = new byte[4];
                msg[0] = 1; // Length
                msg[1] = 7; // Stop Data Stream ID
                serial_port.Write(msg, 0, msg.Length);
            }
        }

        internal void SendStartMission()
        {
            if (serial_port != null && serial_port.IsOpen)
            {
                byte[] msg = new byte[4];
                msg[0] = 1; // Length
                msg[1] = 8; // Start Data Stream ID
                serial_port.Write(msg, 0, msg.Length);
            }
        }

        internal void SendStopMission()
        {
            if (serial_port != null && serial_port.IsOpen)
            {
                byte[] msg = new byte[4];
                msg[0] = 1; // Length
                msg[1] = 9; // Stop Data Stream ID
                serial_port.Write(msg, 0, msg.Length);
            }
        }

        internal void SendCalibrate()
        {
            if (serial_port != null && serial_port.IsOpen)
            {
                byte[] msg = new byte[4];
                msg[0] = 1; // Length
                msg[1] = 1; // Calibrate ID
                serial_port.Write(msg, 0, msg.Length);
            }
        }

        internal void OpenPort(int v)
        {
            serial_port = new SerialPort(serial_port_names[v], 12000000, Parity.None, 8, StopBits.One);
            serial_port.Open();
        }

        public void SetControls(Label x, 
                                Label y, 
                                Label z, 
                                Label rx, 
                                Label ry, 
                                Label rz, 
                                RadioButton rb1, 
                                RadioButton rb2, 
                                RadioButton frb, 
                                RadioButton drb, 
                                RadioButton lrb, 
                                TextBlock log,
                                Dispatcher d)
        {
            accel_x = x;
            accel_y = y;
            accel_z = z;
            rads_x = rx;
            rads_y = ry;
            rads_z = rz;
            ready_rb1 = rb1;
            ready_rb2 = rb2;
            flight_rb = frb;
            deploy_rb = drb;
            landed_rb = lrb;
            log_screen = log;
            dispatcher = d;
        }

        public void ProcessMessage(int message_id, byte[] message)
        {
            switch ((MessageId)message_id)
            {
                case MessageId.SENSOR_DATA:
                    ProcessSensorData(message);
                    break;

                case MessageId.LOG_DATA:

                    break;

                default:
                    break;
            }
        }

        static float ConvertAccel(int accel)
        {
            return (accel * 64.0F) / 0xFFFF;
        }

        static float ConvertRads(int rads)
        {
            return (rads * 2000.0F) / 0xFFFF;
        }

        static int ConvertRawSample(int msb, int lsb)
        {
            return (msb << 8) | lsb;
        }

        private void ProcessSensorData(byte[] message)
        {
            int i = 0;
            int s = ConvertRawSample(message[i+1], message[i+2]);
            i += 2;
            float xf = ConvertAccel(s);
            s = ConvertRawSample(message[i + 1], message[i + 2]);
            i += 2;
            float yf = ConvertAccel(s);
            s = ConvertRawSample(message[i + 1], message[i + 2]);
            i += 2;
            float zf = ConvertAccel(s);
            s = ConvertRawSample(message[i + 1], message[i + 2]);
            i += 2;
            float rxf = ConvertAccel(s);
            s = ConvertRawSample(message[i + 1], message[i + 2]);
            i += 2;
            float ryf = ConvertAccel(s);
            s = ConvertRawSample(message[i + 1], message[i + 2]);
            i += 2;
            float rzf = ConvertAccel(s);

            dispatcher.BeginInvoke(new Action(() => 
            { 
                accel_x.Content = xf.ToString();
                accel_y.Content = yf.ToString();
                accel_z.Content = zf.ToString();
                rads_x.Content = rxf.ToString();
                rads_y.Content = ryf.ToString();
                rads_z.Content = rzf.ToString();
            }));
        }
    }
}
