#include <iostream>

namespace Move3D{
    constexpr double EPS = 1e-10;
    struct Vector{
        double x,y,z;
        Vector() = default;
        Vector(double _x,double _y,double _z){
            x = _x,y = _y,z = _z;
        }
        Vector operator*(double ratio) const {
            return Vector(x * ratio,y * ratio,z * ratio);
        }
        double GetLen() const {
            return sqrt(x * x + y * y + z * z);
        }
    };
    struct Point{
        double x,y,z;
        Point() = default;
        Point(double _x,double _y,double _z){
            x = _x,y = _y,z = _z;
        }
        Point operator+(Vector delta) const {
            return Point(this->x + delta.x,this->y + delta.y,this->z + delta.z);
        }
        Vector ToVector() const {
            return Vector(x,y,z);
        }
    };
    struct RPY{
        double roll,pitch,yaw;
        RPY() = default;
        RPY(double _roll,double _pitch,double _yaw){
            roll = _roll,pitch = _pitch,yaw = _yaw;
        }
    };

    Vector operator-(const Point& p1,const Point& p2){
        return Vector(p1.x - p2.x,p1.y - p2.y,p1.z - p2.z);
    }

    inline double GetDistance(const Point& p1,const Point& p2){
        return sqrt(
            (p1.x - p2.x) * (p1.x - p2.x) + 
            (p1.y - p2.y) * (p1.y - p2.y) + 
            (p1.z - p2.z) * (p1.z - p2.z)
        );
    }

    //WARNING:AI-GEN
    inline Vector RotateRPY(const Vector& v,const RPY& rpy){
        double cr = std::cos(rpy.roll),  sr = std::sin(rpy.roll);
        double cp = std::cos(rpy.pitch), sp = std::sin(rpy.pitch);
        double cy = std::cos(rpy.yaw),   sy = std::sin(rpy.yaw);

        // Rx(roll)
        double x1 = v.x;
        double y1 = v.y * cr - v.z * sr;
        double z1 = v.y * sr + v.z * cr;

        // Ry(pitch)
        double x2 = x1 * cp + z1 * sp;
        double y2 = y1;
        double z2 = -x1 * sp + z1 * cp;

        // Rz(yaw)
        double x3 = x2 * cy - y2 * sy;
        double y3 = x2 * sy + y2 * cy;
        double z3 = z2;

        return Vector(x3,y3,z3);
    }

    inline RPY VectorToRPY(const Vector& v){
        double len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        if(len <= EPS) return RPY(0,0,0);

        double vx = v.x / len,vy = v.y / len,vz = v.z / len;
        double yaw = atan2(vy,vx);
        double pitch = atan2(-vz,sqrt(vx*vx + vy*vy));

        return RPY(0,pitch,yaw);
    }

    class Movement{
    protected:
        Point nowPosition;
    public:
        virtual ~Movement() = default;
        virtual Point Predict(double dt) const = 0;
        Point GetNowPosition() const {
            return nowPosition;
        }
    };

    class UniformLinearMotion : public Movement{
    private:
        Vector velocity;
    public:
        UniformLinearMotion(Point _nowPosition,Vector _velocity){
            nowPosition = _nowPosition;
            velocity = _velocity;
        }
        Point Predict(double dt) const override {
            return nowPosition + velocity * dt;
        }
    };

    class UniformAcceleratedMotion : public Movement{
    private:
        Vector velocity,acceleration;
    public:
        UniformAcceleratedMotion(Point _nowPosition,Vector _velocity,Vector _acceleration){
            nowPosition = _nowPosition;
            velocity = _velocity;
            acceleration = _acceleration;
        }
        Point Predict(double dt) const override {
            return nowPosition + velocity * dt + acceleration * 0.5 * dt * dt;
        }
    };
}//namespace Move3D

inline std::ostream& operator<<(std::ostream& os,const Move3D::Point& _point){
    os<<"P("<<_point.x<<','<<_point.y<<','<<_point.z<<')';
    return os;
}

inline std::ostream& operator<<(std::ostream& os,const Move3D::Vector& _vector){
    os<<"V("<<_vector.x<<','<<_vector.y<<','<<_vector.z<<')';
    return os;
}

inline std::ostream& operator<<(std::ostream& os,const Move3D::RPY& _rpy){
    os<<"RPY("<<_rpy.roll<<','<<_rpy.pitch<<','<<_rpy.yaw<<')';
    return os;
}