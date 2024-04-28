using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Obstacles {    
    public class Obstacle {
        public string name;
        public Vector3 vector3;
        public Transform transform;
        public Obstacle(string name, Vector3 vector3, Transform transform=null){
            this.name = name;
            this.vector3 = vector3;
            this.transform = transform;
        }

        public override bool Equals(object obj) {
            // False if null or different type
            if (obj == null || GetType() != obj.GetType())
                return false;

            Obstacle o = (Obstacle) obj;
            return name == o.name && vector3 == o.vector3;
        }
        
        public override int GetHashCode() {
            return name.GetHashCode() ^ vector3.GetHashCode();
        }
    }

    public class Subgoal {
        public Vector3 position;
        public float distanceToTarget;
        public GameObject gameObj;
        public Subgoal(Vector3 position, float distanceToTarget){
            this.position = position;
            this.distanceToTarget = distanceToTarget;
        }
    }

    public class DynaObstacle {
        public string name;
        public Vector3 vector3;
        public Transform transform;
        public List<Subgoal> subgoalsCandidates = new List<Subgoal>();
        public DynaObstacle(string name, Vector3 vector3, Transform transform=null){
            this.name = name;
            this.vector3 = vector3;
            this.transform = transform;
        }

        public override bool Equals(object obj) {
            // False if null or different type
            if (obj == null || GetType() != obj.GetType())
                return false;

            DynaObstacle o = (DynaObstacle) obj;
            return name == o.name && vector3 == o.vector3;
        }
        
        public override int GetHashCode() {
            return name.GetHashCode() ^ vector3.GetHashCode();
        }
    }

}