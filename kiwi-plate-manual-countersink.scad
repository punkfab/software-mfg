$fn = 30;
m3 = 3;
d = 20;
t = 3;

module countersink_m3() {
   translate([0,0,-t/2]) cylinder(d=m3, h=t);
   translate([0,0,t/2-1.6]) cylinder(h=1.6, r1=5.5, r2=m3);
}

module countersink_m3_bottom() {
   translate([0,0,-t/2]) cylinder(d=m3, h=t);
   translate([0,0,-t/2]) cylinder(h=1.6, d2=3.0, d1=5.5);
}

module plate() {
  cube([34,31,t], center=true);
}

module holes() {
  translate([-d/2, 0]) countersink_m3_bottom();
  translate([d/2, 0]) countersink_m3_bottom();
}


difference() {
hull() {
for(i = [0:2]) {
  rotate([0,0,i*120]) translate([30, 20]) plate();
 }
}
for(i = [0:2]) {
  rotate([0,0,i*120]) translate([30, 20]) holes();
}
}



