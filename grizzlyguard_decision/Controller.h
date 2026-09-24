
class Controller
{
    /*the controller class moves the robot based onwhether an obstacle has been detected, */
public:
    Controller();
    ~Controller();

    // this class should be completed by next week, we need to be able to move the system. 
    int moveDrone(int steering_angle, int speed, int direction);
    bool sprayBear(bool bearDetected, int );
    

    

protected:

private:

};

