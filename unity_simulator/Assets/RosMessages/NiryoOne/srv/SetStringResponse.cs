//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    [Serializable]
    public class SetStringResponse : Message
    {
        public const string k_RosMessageName = "niryo_one_msgs/SetString";
        public override string RosMessageName => k_RosMessageName;

        public int status;
        public string message;

        public SetStringResponse()
        {
            this.status = 0;
            this.message = "";
        }

        public SetStringResponse(int status, string message)
        {
            this.status = status;
            this.message = message;
        }

        public static SetStringResponse Deserialize(MessageDeserializer deserializer) => new SetStringResponse(deserializer);

        private SetStringResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.status);
            deserializer.Read(out this.message);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.status);
            serializer.Write(this.message);
        }

        public override string ToString()
        {
            return "SetStringResponse: " +
            "\nstatus: " + status.ToString() +
            "\nmessage: " + message.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
