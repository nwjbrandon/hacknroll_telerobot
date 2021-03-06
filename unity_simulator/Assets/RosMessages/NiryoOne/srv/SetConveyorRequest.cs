//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    [Serializable]
    public class SetConveyorRequest : Message
    {
        public const string k_RosMessageName = "niryo_one_msgs/SetConveyor";
        public override string RosMessageName => k_RosMessageName;

        public byte id;
        public bool activate;

        public SetConveyorRequest()
        {
            this.id = 0;
            this.activate = false;
        }

        public SetConveyorRequest(byte id, bool activate)
        {
            this.id = id;
            this.activate = activate;
        }

        public static SetConveyorRequest Deserialize(MessageDeserializer deserializer) => new SetConveyorRequest(deserializer);

        private SetConveyorRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.id);
            deserializer.Read(out this.activate);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.id);
            serializer.Write(this.activate);
        }

        public override string ToString()
        {
            return "SetConveyorRequest: " +
            "\nid: " + id.ToString() +
            "\nactivate: " + activate.ToString();
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
