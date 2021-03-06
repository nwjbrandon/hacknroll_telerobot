//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    [Serializable]
    public class GetWorkspaceListResponse : Message
    {
        public const string k_RosMessageName = "niryo_one_msgs/GetWorkspaceList";
        public override string RosMessageName => k_RosMessageName;

        public string[] workspaces;

        public GetWorkspaceListResponse()
        {
            this.workspaces = new string[0];
        }

        public GetWorkspaceListResponse(string[] workspaces)
        {
            this.workspaces = workspaces;
        }

        public static GetWorkspaceListResponse Deserialize(MessageDeserializer deserializer) => new GetWorkspaceListResponse(deserializer);

        private GetWorkspaceListResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.workspaces, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.workspaces);
            serializer.Write(this.workspaces);
        }

        public override string ToString()
        {
            return "GetWorkspaceListResponse: " +
            "\nworkspaces: " + System.String.Join(", ", workspaces.ToList());
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
