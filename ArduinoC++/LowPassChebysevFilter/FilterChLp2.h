// Low pass chebyshev filter order=2 alpha1=0.445
class FilterChLp2
{
  public:
    FilterChLp2()
    {
      v[0]=0.0;
      v[1]=0.0;
      v[2]=0.0;
    }
    
    double addData(double x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (8.319393674533791527e-1 * x)
         + (-0.73546840593695905763 * v[0])
         + (-1.59228906387655766430 * v[1]);
      return (v[0] + v[2]) + 2 * v[1];
    }
    
  private:
    double v[3];
};