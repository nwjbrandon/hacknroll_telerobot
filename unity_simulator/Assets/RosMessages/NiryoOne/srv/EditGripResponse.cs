//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    [Serializable]
    public class EditGripResponse : Message
    {
        public const string k_RosMessageName = "niryo_one_msgs/EditGrip";
        public override string RosMessageName => k_RosMessageName;

        public int status;
        public string message;

        public EditGripResponse()
        {
            this.status = 0;
            this.message = "";
        }

        public EditGripResponse(int status, string message)
        {
            this.status = status;
            this.message = message;
        }

        public static EditGripResponse Deserialize(MessageDeserializer deserializer) => new EditGripResponse(deserializer);

        private EditGripResponse(MessageDeserializer deserializer)
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
            return "EditGripResponse: " +
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
