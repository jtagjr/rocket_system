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
        RadioButton cal_rb;
        RadioButton streaming_rb;
        RadioButton flight_rb;
        RadioButton deploy_rb;
        RadioButton landed_rb;
        Label temp;
        Label pressure;
        Label gpsFix;
        Label ecefx;
        Label ecefy;
        Label ecefz;
        Label accuracy_position;
        Label ecevx;
        Label ecevy;
        Label ecevz;
        Label accuracy_speed;
        Dispatcher dispatcher;
        string[] serial_port_names;

        enum MessageId
        {
            SENSOR_DATA = 20,
            LOG_DATA,
        }

        public MissionControl()
        {
            this_ = this;
            ThreadStart thread_delegate = new ThreadStart(MissionControl.SerialReader);
            Thread serial_task = new Thread(thread_delegate);
            serial_task.Start();
        }

        public void SetControls(Label x, 
                         Label y, 
                         Label z, 
                         Label rx, 
                         Label ry, 
                         Label rz, 
                         RadioButton cal, 
                         RadioButton streaming,
                         RadioButton flight,
                         RadioButton deploy, 
                         RadioButton landed, 
                         Label t,
                         Label p,
                         Label gf, 
                         Label efx,
                         Label efy, 
                         Label efz, 
                         Label aposition, 
                         Label evx, 
                         Label evy, 
                         Label vz, 
                         Label aspeed, 
                         Dispatcher dis)
        {
            accel_x = x;
            accel_y = y;
            accel_z = z;
            rads_x = rx;
            rads_y = ry;
            rads_z = rz;
            cal_rb = cal;
            streaming_rb = streaming;
            flight_rb = flight;
            deploy_rb = deploy;
            landed_rb = landed;
            temp = t;
            pressure = p;
            gpsFix = gf;
            ecefx = efx;
            ecefy = efy;
            ecefz = efz;
            accuracy_position = aposition;
            ecevx = evx;
            ecevy = evy;
            ecevz = vz;
            accuracy_speed = aspeed;
            dispatcher = dis;
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
                if (length - 1 > 0)
                {
                    var message = new byte[length];
                    this_.serial_port.Read(message, 0, length);
                    this_.ProcessMessage(message_id, message);
                } else
                {
                    this_.ProcessMessage(message_id, null);
                }
                
            }
        }

        internal void SendCalibrate()
        {
            if (serial_port != null && serial_port.IsOpen)
            {
                byte[] msg = new byte[2];
                msg[0] = 1; // Length
                msg[1] = 1; // Calibrate ID
                serial_port.Write(msg, 0, msg.Length);
            }
        }

        internal void SendStartDataStream()
        {
            if (serial_port != null && serial_port.IsOpen)
            {
                byte[] msg = new byte[2];
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
                byte[] msg = new byte[2];
                msg[0] = 1; // Length
                msg[1] = 8; // Start Data Stream ID
                serial_port.Write(msg, 0, msg.Length);
            }
        }

        internal void SendStopMission()
        {
            if (serial_port != null && serial_port.IsOpen)
            {
                byte[] msg = new byte[2];
                msg[0] = 1; // Length
                msg[1] = 9; // Stop Data Stream ID
                serial_port.Write(msg, 0, msg.Length);
            }
        }

        internal void OpenPort(int v)
        {
            serial_port = new SerialPort(serial_port_names[v], 12000000, Parity.None, 8, StopBits.One);
            serial_port.Open();
        }

        public void ProcessMessage(int message_id, byte[] message)
        {
            switch ((MessageId)message_id)
            {
                case MessageId.SENSOR_DATA:
                    ProcessSensorData(message);
                    break;
                break;

                default:
                    dispatcher.BeginInvoke(new Action(() =>
                    {
                        switch (message_id)
                        {
                            case 1:
                                cal_rb.IsChecked = true;
                                break;
                            case 2:
                                cal_rb.IsChecked = false;
                                break;
                            case 6:
                                streaming_rb.IsChecked = true;
                                break;
                            case 7:
                                streaming_rb.IsChecked = false;
                                break;
                            case 8:
                                flight_rb.IsChecked = true;
                                break;
                            case 9:
                                flight_rb.IsChecked = false;
                                break;
                        }
                    }));
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

        static Int16 ConvertRawSample(byte msb, byte lsb)
        {
            return (short)((msb << 8) | lsb);
        }

        static Int32 ConvertRawSample(byte b1, byte b2, byte b3, byte b4)
        {
            return (b1 << 24) | (b2 << 16) | (b3 << 8) | b4;
        }

        void DisplayValues(Int16 ax, Int16 ay, Int16 az, Int16 rx, Int16 ry, Int16 rz)
        {
            float xf = ConvertAccel(ax);
            float yf = ConvertAccel(ay);
            float zf = ConvertAccel(az);
            float rxf = ConvertRads(rx);
            float ryf = ConvertRads(ry);
            float rzf = ConvertRads(rz);
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

        private void ProcessSensorData(byte[] message)
        {
            int i = 0; // skip tag and pad

            uint tag1 = message[i++]; 
            Int16 s1 = ConvertRawSample(message[i], message[i + 1]);
            i += 2;
            Int16 s2 = ConvertRawSample(message[i], message[i + 1]);
            i += 2;
            Int16 s3 = ConvertRawSample(message[i], message[i + 1]);
            i += 3; // skip tag
            Int16 s4 = ConvertRawSample(message[i], message[i + 1]);
            i += 2;
            Int16 s5 = ConvertRawSample(message[i], message[i + 1]);
            i += 2;
            Int16 s6 = ConvertRawSample(message[i], message[i + 1]);
            i += 2;
            // Check if Gryo
            if (tag1 == 1)
            {
                DisplayValues(s4, s5, s6, s1, s2, s3);
            } 
            else
            {
                DisplayValues(s1, s2, s3, s4, s5, s6);
            }

            var t = ConvertRawSample(message[i++], message[i++], message[i++], message[i++]) / 1000.0;
            var p = ConvertRawSample(message[i++], message[i++], message[i++], message[i++]) / 1000.0;
            uint gf = message[i++];
            Int32 efx = ConvertRawSample(message[i++], message[i++], message[i++], message[i++]);
            Int32 efy = ConvertRawSample(message[i++], message[i++], message[i++], message[i++]);
            Int32 efz = ConvertRawSample(message[i++], message[i++], message[i++], message[i++]);
            UInt32 aposition = (UInt32)ConvertRawSample(message[i++], message[i++], message[i++], message[i++]);
            Int32 evx = ConvertRawSample(message[i++], message[i++], message[i++], message[i++]);
            Int32 evy = ConvertRawSample(message[i++], message[i++], message[i++], message[i++]);
            Int32 evz = ConvertRawSample(message[i++], message[i++], message[i++], message[i++]);
            UInt32 aspeed = (UInt32)ConvertRawSample(message[i++], message[i++], message[i++], message[i++]);
            
            dispatcher.BeginInvoke(new Action(() => 
            {
                temp.Content = t.ToString();
                pressure.Content = p.ToString();
                gpsFix.Content = gf + " satellites";
                ecefx.Content = efx.ToString() + " cm";
                ecefy.Content = efy.ToString() + " cm";
                ecefz.Content = efz.ToString() + " cm";
                accuracy_position.Content = string.Format("3D position accuracy estimate {0} cm", aposition);
                ecevx.Content = evx.ToString() + " cm/s";
                ecevy.Content = evy.ToString() + " cm/s";
                ecevz.Content = evz.ToString() + " cm/s";
                accuracy_speed.Content = string.Format("3D speed accuracy estimate {0} cm/s", aspeed);
            }));
            
        }
    }
}

