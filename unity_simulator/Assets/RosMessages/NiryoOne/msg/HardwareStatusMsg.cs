//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    [Serializable]
    public class HardwareStatusMsg : Message
    {
        public const string k_RosMessageName = "niryo_one_msgs/HardwareStatus";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        //  Raspberry Pi board
        public int rpi_temperature;
        //  Robot version : 1 (previous one) or 2 (current one)
        public int hardware_version;
        //  Motors
        public bool connection_up;
        public string error_message;
        public int calibration_needed;
        public bool calibration_in_progress;
        public string[] motor_names;
        public string[] motor_types;
        public int[] temperatures;
        public double[] voltages;
        public int[] hardware_errors;

        public HardwareStatusMsg()
        {
            this.header = new Std.HeaderMsg();
            this.rpi_temperature = 0;
            this.hardware_version = 0;
            this.connection_up = false;
            this.error_message = "";
            this.calibration_needed = 0;
            this.calibration_in_progress = false;
            this.motor_names = new string[0];
            this.motor_types = new string[0];
            this.temperatures = new int[0];
            this.voltages = new double[0];
            this.hardware_errors = new int[0];
        }

        public HardwareStatusMsg(Std.HeaderMsg header, int rpi_temperature, int hardware_version, bool connection_up, string error_message, int calibration_needed, bool calibration_in_progress, string[] motor_names, string[] motor_types, int[] temperatures, double[] voltages, int[] hardware_errors)
        {
            this.header = header;
            this.rpi_temperature = rpi_temperature;
            this.hardware_version = hardware_version;
            this.connection_up = connection_up;
            this.error_message = error_message;
            this.calibration_needed = calibration_needed;
            this.calibration_in_progress = calibration_in_progress;
            this.motor_names = motor_names;
            this.motor_types = motor_types;
            this.temperatures = temperatures;
            this.voltages = voltages;
            this.hardware_errors = hardware_errors;
        }

        public static HardwareStatusMsg Deserialize(MessageDeserializer deserializer) => new HardwareStatusMsg(deserializer);

        private HardwareStatusMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.rpi_temperature);
            deserializer.Read(out this.hardware_version);
            deserializer.Read(out this.connection_up);
            deserializer.Read(out this.error_message);
            deserializer.Read(out this.calibration_needed);
            deserializer.Read(out this.calibration_in_progress);
            deserializer.Read(out this.motor_names, deserializer.ReadLength());
            deserializer.Read(out this.motor_types, deserializer.ReadLength());
            deserializer.Read(out this.temperatures, sizeof(int), deserializer.ReadLength());
            deserializer.Read(out this.voltages, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.hardware_errors, sizeof(int), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.rpi_temperature);
            serializer.Write(this.hardware_version);
            serializer.Write(this.connection_up);
            serializer.Write(this.error_message);
            serializer.Write(this.calibration_needed);
            serializer.Write(this.calibration_in_progress);
            serializer.WriteLength(this.motor_names);
            serializer.Write(this.motor_names);
            serializer.WriteLength(this.motor_types);
            serializer.Write(this.motor_types);
            serializer.WriteLength(this.temperatures);
            serializer.Write(this.temperatures);
            serializer.WriteLength(this.voltages);
            serializer.Write(this.voltages);
            serializer.WriteLength(this.hardware_errors);
            serializer.Write(this.hardware_errors);
        }

        public override string ToString()
        {
            return "HardwareStatusMsg: " +
            "\nheader: " + header.ToString() +
            "\nrpi_temperature: " + rpi_temperature.ToString() +
            "\nhardware_version: " + hardware_version.ToString() +
            "\nconnection_up: " + connection_up.ToString() +
            "\nerror_message: " + error_message.ToString() +
            "\ncalibration_needed: " + calibration_needed.ToString() +
            "\ncalibration_in_progress: " + calibration_in_progress.ToString() +
            "\nmotor_names: " + System.String.Join(", ", motor_names.ToList()) +
            "\nmotor_types: " + System.String.Join(", ", motor_types.ToList()) +
            "\ntemperatures: " + System.String.Join(", ", temperatures.ToList()) +
            "\nvoltages: " + System.String.Join(", ", voltages.ToList()) +
            "\nhardware_errors: " + System.String.Join(", ", hardware_errors.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
