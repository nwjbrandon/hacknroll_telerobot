//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    [Serializable]
    public class SetLedsResponse : Message
    {
        public const string k_RosMessageName = "niryo_one_msgs/SetLeds";
        public override string RosMessageName => k_RosMessageName;

        public int status;
        public string message;

        public SetLedsResponse()
        {
            this.status = 0;
            this.message = "";
        }

        public SetLedsResponse(int status, string message)
        {
            this.status = status;
            this.message = message;
        }

        public static SetLedsResponse Deserialize(MessageDeserializer deserializer) => new SetLedsResponse(deserializer);

        private SetLedsResponse(MessageDeserializer deserializer)
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
            return "SetLedsResponse: " +
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
