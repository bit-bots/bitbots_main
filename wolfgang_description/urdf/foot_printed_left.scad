% scale(1000) import("foot_printed_left.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:

translate([154/2-84,0,3.5/2]){
cube([154, 106, 3.5], center=true);
}
translate([89-19/2,0,3.5/2]){
cube([19, 78, 3.5], center=true);
}

translate([-(100-16/2),0,3.5/2]){
cube([16, 78, 3.5], center=true);
}

translate([0,106/2-16/2,3.5+7/2]){
cube([140,16,7], center=true);
}

translate([-(70-79/2),-106/2+16/2,3.5+7/2]){
cube([79,16,7], center=true);
}

translate([70-20/2,-106/2+44/2,3.5+7/2]){
cube([20,44,7], center=true);
}
translate([20/2-11,-106/2+44/2,3.5+7/2]){
cube([20,44,7], center=true);
}
// cylinder(r=10, h=10, center=true);
// sphere(10);
