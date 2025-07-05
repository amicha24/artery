message ClusterMessage {
    int clusterId;          // Unique identifier for the cluster
    int memberId;          // Unique identifier for the cluster member
    double positionX;      // X-coordinate of the member's position
    double positionY;      // Y-coordinate of the member's position
    double positionZ;      // Z-coordinate of the member's position
    double timestamp;      // Timestamp of the message
    string status;         // Status of the cluster member (e.g., "active", "inactive")
}