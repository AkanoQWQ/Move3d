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

    class Movement{
    protected:
        Point nowPosition;
    public:
        virtual ~Movement() = default;
        virtual Point Predict(double dt) const = 0;
    };

    class UniformLinearMotion : public Move3D::Movement{
    private:
        Vector velocity;
    public:
        UniformLinearMotion(Point _nowPosition,Vector _velocity){
            nowPosition = _nowPosition;
            velocity = _velocity;
        }
        Move3D::Point Predict(double dt) const override {
            return nowPosition + velocity * dt;
        }
    };
}//namespace Move3D

inline std::ostream& operator<<(std::ostream& os,const Move3D::Point& _point){
    os<<'('<<_point.x<<','<<_point.y<<','<<_point.z<<')';
    return os;
}