//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    [Serializable]
    public class PushAirVacuumPumpRequest : Message
    {
        public const string k_RosMessageName = "niryo_one_msgs/PushAirVacuumPump";
        public override string RosMessageName => k_RosMessageName;

        public byte id;
        public short push_air_position;

        public PushAirVacuumPumpRequest()
        {
            this.id = 0;
            this.push_air_position = 0;
        }

        public PushAirVacuumPumpRequest(byte id, short push_air_position)
        {
            this.id = id;
            this.push_air_position = push_air_position;
        }

        public static PushAirVacuumPumpRequest Deserialize(MessageDeserializer deserializer) => new PushAirVacuumPumpRequest(deserializer);

        private PushAirVacuumPumpRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.id);
            deserializer.Read(out this.push_air_position);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.id);
            serializer.Write(this.push_air_position);
        }

        public override string ToString()
        {
            return "PushAirVacuumPumpRequest: " +
            "\nid: " + id.ToString() +
            "\npush_air_position: " + push_air_position.ToString();
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
