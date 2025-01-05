% scale(1000) import("ankle.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
translate([0,0,2]){
cube([48,100,4], center=true);
}

translate([0,50-2,50/2]){
cube([21,4,50], center=true);
}
translate([0,-(50-2),50/2]){
cube([21,4,50], center=true);
}
translate([0,-(50-2),8/2]){
cube([48,4,8], center=true);
}

translate([-10.4,-(50-2),26.7]){
    rotate([0,18.373539,0]){
cube([14,4,(50-8)/cos(18.373539)], center=true);
}
}

translate([10.4,-(50-2),26.7]){
    rotate([0,-18.373539,0]){
cube([14,4,(50-8)/cos(18.373539)], center=true);
}
}
translate([-10.4,(50-2),26.7]){
    rotate([0,18.373539,0]){
cube([14,4,(50-8)/cos(18.373539)], center=true);
}
}

translate([10.4,(50-2),26.7]){
    rotate([0,-18.373539,0]){
cube([14,4,(50-8)/cos(18.373539)], center=true);
}
}
translate([0,(50-2),8/2]){
cube([48,4,8], center=true);
}