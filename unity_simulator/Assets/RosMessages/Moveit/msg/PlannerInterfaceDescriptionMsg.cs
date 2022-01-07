//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Moveit
{
    [Serializable]
    public class PlannerInterfaceDescriptionMsg : Message
    {
        public const string k_RosMessageName = "moveit_msgs/PlannerInterfaceDescription";
        public override string RosMessageName => k_RosMessageName;

        //  The name of the planner interface
        public string name;
        //  The names of the planner ids within the interface
        public string[] planner_ids;

        public PlannerInterfaceDescriptionMsg()
        {
            this.name = "";
            this.planner_ids = new string[0];
        }

        public PlannerInterfaceDescriptionMsg(string name, string[] planner_ids)
        {
            this.name = name;
            this.planner_ids = planner_ids;
        }

        public static PlannerInterfaceDescriptionMsg Deserialize(MessageDeserializer deserializer) => new PlannerInterfaceDescriptionMsg(deserializer);

        private PlannerInterfaceDescriptionMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.name);
            deserializer.Read(out this.planner_ids, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.name);
            serializer.WriteLength(this.planner_ids);
            serializer.Write(this.planner_ids);
        }

        public override string ToString()
        {
            return "PlannerInterfaceDescriptionMsg: " +
            "\nname: " + name.ToString() +
            "\nplanner_ids: " + System.String.Join(", ", planner_ids.ToList());
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
