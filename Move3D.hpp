namespace Move3D{
    struct Point{
        double x,y,z;
    };
    struct Vector{
        double x,y,z;
        Vector(double _x,double _y,double _z){
            x = _x,y = _y,z = _z;
        }
        Vector operator*(double ratio){
            return Vector(x * ratio,y * ratio,z * ratio);
        }
    };

    class Movement{
    protected:
        Point nowPosition;
    public:
        virtual ~Movement() = default;
        virtual Point Move(double dt) = 0;
    };
    
}//namespace Move3D