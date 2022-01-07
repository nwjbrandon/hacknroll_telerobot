//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    [Serializable]
    public class ControlConveyorRequest : Message
    {
        public const string k_RosMessageName = "niryo_one_msgs/ControlConveyor";
        public override string RosMessageName => k_RosMessageName;

        public byte id;
        public bool control_on;
        public short speed;
        public sbyte direction;

        public ControlConveyorRequest()
        {
            this.id = 0;
            this.control_on = false;
            this.speed = 0;
            this.direction = 0;
        }

        public ControlConveyorRequest(byte id, bool control_on, short speed, sbyte direction)
        {
            this.id = id;
            this.control_on = control_on;
            this.speed = speed;
            this.direction = direction;
        }

        public static ControlConveyorRequest Deserialize(MessageDeserializer deserializer) => new ControlConveyorRequest(deserializer);

        private ControlConveyorRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.id);
            deserializer.Read(out this.control_on);
            deserializer.Read(out this.speed);
            deserializer.Read(out this.direction);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.id);
            serializer.Write(this.control_on);
            serializer.Write(this.speed);
            serializer.Write(this.direction);
        }

        public override string ToString()
        {
            return "ControlConveyorRequest: " +
            "\nid: " + id.ToString() +
            "\ncontrol_on: " + control_on.ToString() +
            "\nspeed: " + speed.ToString() +
            "\ndirection: " + direction.ToString();
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
