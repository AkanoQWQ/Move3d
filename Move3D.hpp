#include <iostream>

namespace Move3D{
    struct Vector{
        double x,y,z;
        Vector() = default;
        Vector(double _x,double _y,double _z){
            x = _x,y = _y,z = _z;
        }
        Vector operator*(double ratio) const {
            return Vector(x * ratio,y * ratio,z * ratio);
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
    };
    double GetDistance(const Point& p1,const Point& p2){
        return sqrt(
            (p1.x - p2.x) * (p1.x - p2.x) + 
            (p1.y - p2.y) * (p1.y - p2.y) + 
            (p1.z - p2.z) * (p1.z - p2.z)
        );
    }

    class Movement{
    protected:
        Point nowPosition;
    public:
        virtual ~Movement() = default;
        virtual Point Predict(double dt) const = 0;
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