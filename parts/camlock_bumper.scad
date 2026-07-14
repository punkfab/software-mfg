$fn = 50;

t = 2;
d = 63.5;
pin_d = 8;
engagement_w = 6.5;

for(i = [0:2]) {
  rotate([0,0,i*120])
    translate([26, 0])
    union() {
        translate([0,0,-3]) cylinder(d=pin_d, h=3);
        hull() {
            translate([0,0,engagement_w+t-pin_d/2]) sphere(d=pin_d);
            cylinder(h=engagement_w+t-pin_d/2,d=8);
        }
     }   
}
       cylinder(h=t, d=d);
     