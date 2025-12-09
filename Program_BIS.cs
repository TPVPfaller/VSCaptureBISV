/*
 * This file is part of VitalSignsCaptureBISV v1.003.
 * Copyright (C) 2024 John George K., xeonfusion@users.sourceforge.net

    VitalSignsCaptureBISV is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VitalSignsCaptureBISV is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with VitalSignsCaptureBISV.  If not, see <http://www.gnu.org/licenses/>.*/


using System.Management;
using System.IO.Ports;

namespace VSCaptureBISV
{
    class ProgramBISV
    {
        static EventHandler dataEvent;

        public static void Main(string[] args)
        {
            Console.WriteLine("VitalSignsCaptureBISV v1.003 (C)2024 John George K.");
            Console.WriteLine("For command line usage: -help");
            Console.WriteLine();

            // Create a new SerialPort object with default settings.
            BSerialPort _serialPort = BSerialPort.getInstance;
            string portName;
            string sIntervalset;
            string sWaveformSet;
            string sSpectralset;

            var parser = new CommandLineParser();
            parser.Parse(args);

            if (parser.Arguments.ContainsKey("help"))
            {
                Console.WriteLine("VSCaptureBISV.exe -port[portname] -interval[number] -scale[number]");
                Console.WriteLine("-port <Set serial port name>");
                Console.WriteLine("-interval <Set numeric transmission interval>");
                Console.WriteLine("-scale <Set waveform ADC or calibrated export option>");
                Console.WriteLine();
                return;
            }

            if (parser.Arguments.ContainsKey("port"))
            {
                _serialPort.PortName = parser.Arguments["port"][0];
            }
            else
            {
                _serialPort.PortName = "COM2";
                Console.WriteLine("Available serial ports:");
                foreach (string s in SerialPort.GetPortNames())
                {
                    Console.WriteLine(" {0}", s);
                }
                Console.Write("Type in the serial port name where the monitor is connected (default is {0}):", _serialPort.PortName);
                string portInput = Console.ReadLine();
                if (!string.IsNullOrWhiteSpace(portInput))
                {
                    _serialPort.PortName = portInput.Trim();
                }
            }

            try
            {
                _serialPort.Open();

                if (_serialPort.OSIsUnix())
                {
                    dataEvent += new EventHandler((object sender, EventArgs e) => ReadData(sender));
                }

                if (!_serialPort.OSIsUnix())
                {
                    _serialPort.DataReceived += new SerialDataReceivedEventHandler(p_DataReceived);
                }

                int nInterval = 1;

                string sWavescaleSet;
                if (parser.Arguments.ContainsKey("scale"))
                {
                    sWavescaleSet = parser.Arguments["scale"][0];
                }
                else
                {
                    Console.WriteLine();
                    Console.WriteLine("Waveform data export scale and calibrate options:");
                    Console.WriteLine("1. Export scaled ADC values");
                    Console.WriteLine("2. Export calibrated values");
                    Console.WriteLine();
                    Console.Write("Choose Waveform data export scale option (1-2):");

                    sWavescaleSet = Console.ReadLine();
                }

                short nWavescaleSet = 2;
                if (sWavescaleSet != "") nWavescaleSet = Convert.ToInt16(sWavescaleSet);

                if (nWavescaleSet == 1) _serialPort.m_calibratewavevalues = false;
                if (nWavescaleSet == 2) _serialPort.m_calibratewavevalues = true;


                Console.WriteLine();
                Console.WriteLine("Data will be read from {0} and sent to LSL with name BIS_EEG", _serialPort.PortName);

                //_serialPort.RequestStatus();
                //WaitForMilliSeconds(200);

                Task.Run(() => _serialPort.SendCycledRequests(nInterval));

                Console.WriteLine("Press Escape button to Stop");

                if (_serialPort.OSIsUnix())
                {
                    do
                    {
                        if (_serialPort.BytesToRead != 0)
                        {
                            dataEvent.Invoke(_serialPort, new EventArgs());
                        }

                        if (Console.KeyAvailable == true)
                        {
                            if (Console.ReadKey(true).Key == ConsoleKey.Escape) break;
                        }
                    }
                    while (Console.KeyAvailable == false);
                }

                if (!_serialPort.OSIsUnix())
                {
                    ConsoleKeyInfo cki;

                    do
                    {
                        cki = Console.ReadKey(true);
                    }
                    while (cki.Key != ConsoleKey.Escape);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error opening/writing to serial port :: " + ex.Message, "Error!");
            }
            finally
            {
                _serialPort.StopTransfer();

                _serialPort.Close();
            }
        }

        static void p_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            ReadData(sender);
        }

        public static void ReadData(object sender)
        {
            try
            {
                (sender as BSerialPort).ReadBuffer();
            }
            catch (TimeoutException) { }
        }
    }




    public class CommandLineParser
    {
        public CommandLineParser()
        {
            Arguments = new Dictionary<string, string[]>();
        }

        public IDictionary<string, string[]> Arguments { get; private set; }

        public void Parse(string[] args)
        {
            string currentName = "";
            var values = new List<string>();
            foreach (string arg in args)
            {
                if (arg.StartsWith("-", StringComparison.InvariantCulture))
                {
                    if (currentName != "" && values.Count != 0)
                        Arguments[currentName] = values.ToArray();

                    else
                    {
                        values.Add("");
                        Arguments[currentName] = values.ToArray();
                    }
                    values.Clear();
                    currentName = arg.Substring(1);
                }
                else if (currentName == "")
                    Arguments[arg] = new string[0];
                else
                    values.Add(arg);
            }

            if (currentName != "")
                Arguments[currentName] = values.ToArray();
        }
    }

}
