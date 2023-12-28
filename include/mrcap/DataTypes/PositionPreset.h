class PositionPreset {
public:
    std::string name;
    float startX;
    float startY;
    float startTheta;
    float goalX;
    float goalY;
    float goalTheta;

    PositionPreset(const std::string &n, float sX, float sY, float sTh ,float gX, float gY, float gTh)
        : name(n), startX(sX), startY(sY), startTheta(sTh), goalX(gX), goalY(gY) , goalTheta(gTh) {}
};
