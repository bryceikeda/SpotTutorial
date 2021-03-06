//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Gazebo
{
    [Serializable]
    public class ContactStateMsg : Message
    {
        public const string k_RosMessageName = "gazebo_msgs/ContactState";
        public override string RosMessageName => k_RosMessageName;

        public string info;
        //  text info on this contact
        public string collision1_name;
        //  name of contact collision1
        public string collision2_name;
        //  name of contact collision2
        public Geometry.WrenchMsg[] wrenches;
        //  list of forces/torques
        public Geometry.WrenchMsg total_wrench;
        //  sum of forces/torques in every DOF
        public Geometry.Vector3Msg[] contact_positions;
        //  list of contact position
        public Geometry.Vector3Msg[] contact_normals;
        //  list of contact normals
        public double[] depths;
        //  list of penetration depths

        public ContactStateMsg()
        {
            this.info = "";
            this.collision1_name = "";
            this.collision2_name = "";
            this.wrenches = new Geometry.WrenchMsg[0];
            this.total_wrench = new Geometry.WrenchMsg();
            this.contact_positions = new Geometry.Vector3Msg[0];
            this.contact_normals = new Geometry.Vector3Msg[0];
            this.depths = new double[0];
        }

        public ContactStateMsg(string info, string collision1_name, string collision2_name, Geometry.WrenchMsg[] wrenches, Geometry.WrenchMsg total_wrench, Geometry.Vector3Msg[] contact_positions, Geometry.Vector3Msg[] contact_normals, double[] depths)
        {
            this.info = info;
            this.collision1_name = collision1_name;
            this.collision2_name = collision2_name;
            this.wrenches = wrenches;
            this.total_wrench = total_wrench;
            this.contact_positions = contact_positions;
            this.contact_normals = contact_normals;
            this.depths = depths;
        }

        public static ContactStateMsg Deserialize(MessageDeserializer deserializer) => new ContactStateMsg(deserializer);

        private ContactStateMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.info);
            deserializer.Read(out this.collision1_name);
            deserializer.Read(out this.collision2_name);
            deserializer.Read(out this.wrenches, Geometry.WrenchMsg.Deserialize, deserializer.ReadLength());
            this.total_wrench = Geometry.WrenchMsg.Deserialize(deserializer);
            deserializer.Read(out this.contact_positions, Geometry.Vector3Msg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.contact_normals, Geometry.Vector3Msg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.depths, sizeof(double), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.info);
            serializer.Write(this.collision1_name);
            serializer.Write(this.collision2_name);
            serializer.WriteLength(this.wrenches);
            serializer.Write(this.wrenches);
            serializer.Write(this.total_wrench);
            serializer.WriteLength(this.contact_positions);
            serializer.Write(this.contact_positions);
            serializer.WriteLength(this.contact_normals);
            serializer.Write(this.contact_normals);
            serializer.WriteLength(this.depths);
            serializer.Write(this.depths);
        }

        public override string ToString()
        {
            return "ContactStateMsg: " +
            "\ninfo: " + info.ToString() +
            "\ncollision1_name: " + collision1_name.ToString() +
            "\ncollision2_name: " + collision2_name.ToString() +
            "\nwrenches: " + System.String.Join(", ", wrenches.ToList()) +
            "\ntotal_wrench: " + total_wrench.ToString() +
            "\ncontact_positions: " + System.String.Join(", ", contact_positions.ToList()) +
            "\ncontact_normals: " + System.String.Join(", ", contact_normals.ToList()) +
            "\ndepths: " + System.String.Join(", ", depths.ToList());
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
