% scale(1000) import("camera_deck.stl");

// Sketch PureShapes 3
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 3.000000;
translate([0, 0, -thickness]) {
  translate([124.639556, 99.646300, 0]) {
    rotate([0, 0, 180.0]) {
      cube([249.279112, 199.292600, thickness]);
    }
  }
}
}
