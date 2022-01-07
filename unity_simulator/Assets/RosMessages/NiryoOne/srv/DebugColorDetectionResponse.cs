//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    [Serializable]
    public class DebugColorDetectionResponse : Message
    {
        public const string k_RosMessageName = "niryo_one_msgs/DebugColorDetection";
        public override string RosMessageName => k_RosMessageName;

        public Sensor.CompressedImageMsg img;

        public DebugColorDetectionResponse()
        {
            this.img = new Sensor.CompressedImageMsg();
        }

        public DebugColorDetectionResponse(Sensor.CompressedImageMsg img)
        {
            this.img = img;
        }

        public static DebugColorDetectionResponse Deserialize(MessageDeserializer deserializer) => new DebugColorDetectionResponse(deserializer);

        private DebugColorDetectionResponse(MessageDeserializer deserializer)
        {
            this.img = Sensor.CompressedImageMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.img);
        }

        public override string ToString()
        {
            return "DebugColorDetectionResponse: " +
            "\nimg: " + img.ToString();
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