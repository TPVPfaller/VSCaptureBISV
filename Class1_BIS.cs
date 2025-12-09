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


using SharpLSL;
using System.Text;
using System.IO.Ports;
using System.Globalization;
using System.Diagnostics;
using System.Net;
using System.Text.Json;

namespace VSCaptureBISV
{
    public class Crc
    {
        public ushort ComputeChecksum(byte[] bytes)
        {
            ushort sum = 0;
            ushort crc = 0;

            for (int i = 0; i < bytes.Length; ++i)
            {
                sum += bytes[i];
            }

            //get 16-bit sum total checksum
            crc = (ushort)(sum & 0xFFFF);                  //For BIS Vista
            return crc;
        }
    }

    public sealed class BSerialPort : SerialPort
    {
        private int BPortBufSize;
        public byte[] BPort_rxbuf;
        public List<byte[]> FrameList = new List<byte[]>();
        public List<byte> m_BufferByteList = new List<byte>();
        public List<byte> m_ResponseByteList = new List<byte>();
        public string m_strTimestamp;


        public List<WaveValResult> m_WaveValResultList = new List<WaveValResult>();
        public double m_RealtiveTimeCounter = 0;

        public dsc_info_struct m_Dsc_Info_Struct = new dsc_info_struct();
        public bool m_calibratewavevalues = true;
        public double m_defaultgain = 0.05;
        public double m_defaultoffset = -3234;
        private StreamOutlet eegOutlet;
        private bool lslInitialized = false;

        public class WaveValResult
        {
            public string Timestamp;
            public string Relativetimestamp;
            public string PhysioID;
            public string Value;
            public double Relativetimecounter;
        }

        //Create a singleton serialport subclass
        private static volatile BSerialPort BPort = null;

        public static BSerialPort getInstance
        {
            get
            {
                if (BPort == null)
                {
                    lock (typeof(BSerialPort))
                        if (BPort == null)
                        {
                            BPort = new BSerialPort();
                        }
                }
                return BPort;
            }
        }

        public BSerialPort()
        {
            BPort = this;

            BPortBufSize = 4096;
            BPort_rxbuf = new byte[BPortBufSize];

            if (OSIsUnix())
                BPort.PortName = "/dev/ttyUSB0"; //default Unix port
            else BPort.PortName = "COM1"; //default Windows port

            BPort.BaudRate = 57600;
            BPort.Parity = Parity.None;
            BPort.DataBits = 8;
            BPort.StopBits = StopBits.One;

            BPort.Handshake = Handshake.None;
            //BPort.RtsEnable = true;
            //BPort.DtrEnable = true;

            // Set the read/write timeouts
            BPort.ReadTimeout = 600000;
            BPort.WriteTimeout = 600000;

            //ASCII Encoding in C# is only 7bit so
            BPort.Encoding = Encoding.GetEncoding("ISO-8859-1");
            InitializeLSL();
        }

        private void InitializeLSL()
        {
            if (lslInitialized) return;

            var info = new StreamInfo(
                "BIS_EEG",        // name
                "EEG",            // type
                2,                // channel count
                128,              // sampling rate
                ChannelFormat.Float, // channel format
                "bisvista_eeg_001"
            );

            eegOutlet = new StreamOutlet(info);

            lslInitialized = true;
        }

        public void DebugLine(string msg)
        {
            Debug.WriteLine(DateTime.Now.ToString("hh:mm:ss.fff") + " - " + msg);
        }

        public void RequestRawEEGData()
        {
            BPort.WriteBuffer(DataConstants.poll_request_raw_eeg_data);
            DebugLine("Send: Request Raw EEG Data");
        }

        public void WriteBuffer(byte[] txbuf)
        {
            List<byte> temptxbufflist = new List<byte>();

            int framelen = txbuf.Length;
            if (framelen != 0)
            {
                //byte[] txbuf2 = new byte[framelen-2];
                //Array.Copy(txbuf, 2, txbuf2, 0, framelen - 2);
                temptxbufflist.AddRange(txbuf);

                byte[] inputbuffer = temptxbufflist.ToArray();

                Crc crccheck = new Crc();
                ushort checksumcomputed = crccheck.ComputeChecksum(inputbuffer);

                byte[] checksumarray = BitConverter.GetBytes(checksumcomputed);

                temptxbufflist.AddRange(checksumarray);
                temptxbufflist.InsertRange(0, DataConstants.spi_id);

                byte[] finaltxbuff = temptxbufflist.ToArray();

                try
                {
                    BPort.Write(finaltxbuff, 0, finaltxbuff.Length);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error opening/writing to serial port :: " + ex.Message, "Error!");
                }
            }
        }

        public async Task SendCycledRequests(int nInterval)
        {
            int nmillisecond = nInterval * 1000;
            if (nmillisecond != 0)
            {
                do
                {
                    RequestRawEEGData();
                    await Task.Delay(nmillisecond);
                }
                while (true);
            }
            RequestRawEEGData();
        }

        public void ClearReadBuffer()
        {
            for (int i = 0; i < BPortBufSize; i++)
            {
                BPort_rxbuf[i] = 0;
            }
        }

        public int ReadBuffer()
        {
            int bytesreadtotal = 0;

            try
            {
                string path = Path.Combine(Directory.GetCurrentDirectory(), "BISVRawoutput.raw");

                int lenread = 0;

                do
                {
                    ClearReadBuffer();
                    lenread = BPort.Read(BPort_rxbuf, 0, BPortBufSize);

                    if (lenread != 0)
                    {
                        byte[] copyarray = new byte[lenread];

                        Buffer.BlockCopy(BPort_rxbuf, 0, copyarray, 0, lenread);

                        m_BufferByteList.AddRange(copyarray);
                        byte[] BufferArray = m_BufferByteList.ToArray();
                        //Once buffer array has been saved clear the list to add last segment if needed
                        m_BufferByteList.Clear();

                        List<byte[]> segments = new List<byte[]>();
                        byte[] lastsegment = Array.Empty<byte>();

                        if (BufferArray.Length > 0)
                            lastsegment = SplitArrayByDelimiter(BufferArray, segments);
                        if (lastsegment != null)
                            m_BufferByteList.AddRange(lastsegment);

                        if (segments.Count > 0)
                        {
                            foreach (byte[] segment in segments)
                            {
                                m_ResponseByteList.AddRange(segment);
                                ProcessCompleteFrame();
                            }
                        }

                        //ByteArrayToFile(path, copyarray, copyarray.GetLength(0));
                        bytesreadtotal += lenread;

                    }
                }
                while (BPort.BytesToRead != 0);

                if (BPort.BytesToRead == 0)
                {
                    if (FrameList.Count > 0)
                    {
                        ReadFrameData();
                        FrameList.Clear();
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error opening/writing to serial port :: " + ex.Message, "Error!");
            }
            return bytesreadtotal;
        }

        public byte[] SplitArrayByDelimiter(byte[] originalArray, List<byte[]> segments)
        {
            try
            {
                byte[] delimiter = DataConstants.spi_id;  // Set the desired delimiter

                ReadOnlySpan<byte> originalSpan = new ReadOnlySpan<byte>(originalArray);

                int startIndex = 0;
                int delimiterIndex;

                while ((delimiterIndex = SearchBytes(originalSpan.ToArray(), delimiter, startIndex)) != -1)
                {
                    int segmentLength = delimiterIndex - startIndex;
                    if (segmentLength > 0)
                    {
                        byte[] segment = new byte[segmentLength];
                        originalSpan.Slice(startIndex, segmentLength).CopyTo(segment);
                        segments.Add(segment);
                    }
                    startIndex = delimiterIndex + delimiter.Length;
                }

                // Add the remaining part after the last delimiter
                byte[] lastSegment = new byte[originalArray.Length - startIndex];
                if (lastSegment.Length > 0)
                    originalSpan.Slice(startIndex, lastSegment.Length).CopyTo(lastSegment);

                //if (BitConverter.ToInt16(lastSegment) != BitConverter.ToInt16(DataConstants.spi_id))
                return lastSegment;
                //else return null;
                // Now 'segments' contains the split frames, and 'lastSegment' contains the remaining bytes
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error opening/writing to serial port :: " + ex.Message, "Error!");
                return null;
            }
        }

        public static int SearchBytes(byte[] haystack, byte[] needle, int startindex)
        {
            //KMP Search algorithm
            var len = needle.Length;
            var limit = haystack.Length - len;

            for (var i = startindex; i <= limit; i++)
            {
                var k = 0;
                for (; k < len; k++)
                {
                    if (needle[k] != haystack[i + k]) break;
                }
                if (k == len) return i; // Found the needle at index i in the haystack
            }
            return -1; // Needle not found
        }

        private void ProcessCompleteFrame()
        {
            int framelen = m_ResponseByteList.Count;
            if (framelen >= 2)
            {
                byte[] bArray = m_ResponseByteList.ToArray();

                // Serial data without checksum bytes
                int userdataframelen = framelen - 2;
                byte[] userdataArray = new byte[userdataframelen];
                Array.Copy(bArray, 0, userdataArray, 0, userdataframelen);

                // Calculate checksum
                Crc crccheck = new Crc();
                ushort checksumcomputed = crccheck.ComputeChecksum(userdataArray);

                byte[] bchecksum = new byte[2];
                Array.Copy(bArray, framelen - 2, bchecksum, 0, 2);
                ushort checksum = BitConverter.ToUInt16(bchecksum, 0);

                if (checksumcomputed == checksum)
                {
                    FrameList.Add(userdataArray);
                }
                else
                {
                    Console.WriteLine("Checksum Error");
                }
                m_ResponseByteList.Clear();
            }
        }

        public void ReadFrameData()
        {
            if (FrameList.Count > 0)
            {
                foreach (byte[] fArray in FrameList)
                {
                    ProcessPacket(fArray);
                }
            }
        }

        public void ProcessPacket(byte[] packetbuffer)
        {
            if (packetbuffer.Length != 0)
            {
                MemoryStream memstream = new MemoryStream(packetbuffer);
                BinaryReader binreader = new BinaryReader(memstream);

                uint packetseqid = binreader.ReadUInt16();
                uint odlen = binreader.ReadUInt16();
                uint ldpackettype = binreader.ReadUInt16();

                byte[] datapacket = binreader.ReadBytes(packetbuffer.Length - 6);

                switch (ldpackettype)
                {
                    case DataConstants.L1_DATA_PACKET:
                        ReadDataPacket(datapacket);
                        break;
                    case DataConstants.L1_ACK_PACKET:
                        //ACK
                        break;
                    case DataConstants.L1_NAK_PACKET:
                        //NAK
                        break;
                    default:
                        break;
                }
            }
        }

        public void ReadDataPacket(byte[] datapacketbuffer)
        {
            if (datapacketbuffer.Length != 0)
            {
                m_strTimestamp = DateTime.Now.ToString("dd-MM-yyyy HH:mm:ss.fff", CultureInfo.InvariantCulture);

                MemoryStream memstream = new MemoryStream(datapacketbuffer);
                BinaryReader binreader = new BinaryReader(memstream);

                uint routingid = binreader.ReadUInt32();
                uint messageid = binreader.ReadUInt32();
                uint seqnum = binreader.ReadUInt16();
                uint messagelen = binreader.ReadUInt16();

                byte[] messagedata = binreader.ReadBytes(datapacketbuffer.Length - 12);

                switch (messageid)
                {
                    case DataConstants.M_DATA_RAW_EEG:
                        ReadRawEEGDataPacket(messagedata);
                        break;
                    default:
                        break;
                }
            }
        }

        public double ScaleADCValue(short Waveval)
        {
            dsc_info_struct dscscaledata = m_Dsc_Info_Struct;
            if (!double.IsNaN(Waveval))
            {
                double gain = m_defaultgain;
                double offset = m_defaultoffset;
                double value = 0;

                //Get value from 16 bit ADC values using offset and gain
                if (dscscaledata != null && dscscaledata.dsc_gain_divisor != 0 && dscscaledata.dsc_offset_divisor != 0)
                {
                    gain = (double)dscscaledata.dsc_gain_num / dscscaledata.dsc_gain_divisor;
                    offset = (double)dscscaledata.dsc_offset_num / dscscaledata.dsc_offset_divisor;
                }

                value = (double)gain * (Waveval - offset);
                value = Math.Round(value, 2);

                return value;
            }
            else return Waveval;
        }

        public void ReadRawEEGDataPacket(byte[] rawdatapacketbuffer)
        {
            if (rawdatapacketbuffer.Length != 0)
            {
                MemoryStream memstream = new MemoryStream(rawdatapacketbuffer);
                BinaryReader binreader = new BinaryReader(memstream);

                int nchannels = binreader.ReadInt16();
                int nspeed = binreader.ReadInt16();

                int raweegdatalen = (rawdatapacketbuffer.Length - 4);
                byte[] raweegdata = binreader.ReadBytes(raweegdatalen);
                byte[] eegch1 = new byte[2];
                byte[] eegch2 = new byte[2];

                for (int i = 0; i < raweegdatalen; i = i + 4)
                {
                    Array.Copy(raweegdata, i, eegch1, 0, 2);
                    Array.Copy(raweegdata, i + 2, eegch2, 0, 2);

                    //short eegch1data = TwosComplementToInt16(eegch1);
                    //short eegch2data = TwosComplementToInt16(eegch2);

                    short eegch1data = BitConverter.ToInt16(eegch1, 0);
                    short eegch2data = BitConverter.ToInt16(eegch2, 0);

                    WaveValResult WaveVal1 = new WaveValResult();

                    WaveVal1.Relativetimecounter = m_RealtiveTimeCounter;
                    WaveVal1.Relativetimestamp = m_RealtiveTimeCounter.ToString(CultureInfo.InvariantCulture);

                    WaveVal1.Timestamp = m_strTimestamp;
                    WaveVal1.PhysioID = "EEG1";

                    if (m_calibratewavevalues == true)
                    {
                        //Scale and Range Value in ADC
                        double eegch1val = ScaleADCValue(eegch1data);
                        WaveVal1.Value = eegch1val.ToString(CultureInfo.InvariantCulture);
                    }
                    else WaveVal1.Value = eegch1data.ToString(CultureInfo.InvariantCulture);

                    m_WaveValResultList.Add(WaveVal1);

                    WaveValResult WaveVal2 = new WaveValResult();

                    WaveVal2.Relativetimecounter = m_RealtiveTimeCounter;
                    WaveVal2.Relativetimestamp = m_RealtiveTimeCounter.ToString(CultureInfo.InvariantCulture);

                    WaveVal2.Timestamp = m_strTimestamp;
                    WaveVal2.PhysioID = "EEG2";

                    if (m_calibratewavevalues == true)
                    {
                        //Scale and Range Value in ADC
                        double eegch2val = ScaleADCValue(eegch2data);
                        WaveVal2.Value = eegch2val.ToString(CultureInfo.InvariantCulture);

                    }
                    else WaveVal2.Value = eegch2data.ToString(CultureInfo.InvariantCulture);

                    m_WaveValResultList.Add(WaveVal2);

                    //nspeed eeg packets are read every sec
                    m_RealtiveTimeCounter = (m_RealtiveTimeCounter + (1 / (double)nspeed)); //sec
                    m_RealtiveTimeCounter = Math.Round(m_RealtiveTimeCounter, 3);

                    if (lslInitialized)
                    {
                        float v1, v2;

                        if (m_calibratewavevalues)
                        {
                            v1 = (float)ScaleADCValue(eegch1data);
                            v2 = (float)ScaleADCValue(eegch2data);
                        }
                        else
                        {
                            v1 = eegch1data;
                            v2 = eegch2data;
                        }

                        float[] sample = new float[] { v1, v2 };
                        eegOutlet.PushSample(sample);
                    }
                }
            }
        }

        public void StopTransfer()
        {
            WriteBuffer(DataConstants.poll_stop_processed_data);
            DebugLine("Send: Stop Processed Data");
            WriteBuffer(DataConstants.poll_stop_raw_eeg_data);
            DebugLine("Send: Stop Raw EEG Data");
            this.Dispose();
        }

        public bool ByteArrayToFile(string _FileName, byte[] _ByteArray, int nWriteLength)
        {
            try
            {
                // Open file for reading. 
                using (FileStream _FileStream = new FileStream(_FileName, FileMode.Append, FileAccess.Write))
                {
                    // Writes a block of bytes to this stream using data from a byte array
                    _FileStream.Write(_ByteArray, 0, nWriteLength);

                    // close file stream. 
                    _FileStream.Close();
                }
                return true;
            }

            catch (Exception _Exception)
            {
                // Error. 
                Console.WriteLine("Exception caught in process: {0}", _Exception.ToString());
            }
            // error occured, return false. 
            return false;
        }


        public bool OSIsUnix()
        {
            int p = (int)Environment.OSVersion.Platform;
            if ((p == 4) || (p == 6) || (p == 128)) return true;
            else return false;
        }
    }
}

