namespace Move3D{
    struct Point{double x,y,z;};
    struct Vector{double x,y,z;};

    class Movement{
    private:
        Point nowPosition;
    public:
        virtual ~Movement() = default;
        virtual Point Move(double dt) = 0;
    };
    
}//namespace Move3D