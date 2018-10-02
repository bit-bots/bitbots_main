#ifndef ROBOT_STATE
#define ROBOT_STATE

class RobotState
{
public:
    RobotState();
    ~RobotState();

    // RobotState operator*(float factor) const;

    // RobotState& operator+=(const RobotState& other);


    float getXPos() const;

    void setXPos(float x);

    float getYPos() const;

    void setYPos(float y);

    // float getTheta() const;

    // void setTheta(float t);

    // float getSpeed() const;

    // void setSpeed(float s);

    // float getRotationSpeed() const;

    // void setRotationSpeed(float s);


protected:

    float xPos_;
    float yPos_;
};

/*
 * RobotStateW - the robot state with width
 */

class RobotStateW :  RobotState
{
public:
    RobotStateW();
    ~RobotStateW();

    float getWidth() const;

    void setWidth(float t);

private:

    float width_;
};


/*
 * RobotStateO - the robot state with orientation
 */

class RobotStateO :  RobotState
{
public:
    RobotStateO();
    ~RobotStateO();

    float getOrientation() const;

    void setOrientation(float t);

private:

    float orientation_;
};

#endif

