#! /usr/bin/perl
use strict;

# Robot
my $robot_ip = undef;
#my $robot_ip = '192.168.0.103';

# Step 1
my $increments_per_tour = 4198.4;

# Step 2
my $axis_wheel_ratio = 9053.00992 / $increments_per_tour;

# Step 3 (in meters)
my $wheel_diameter = 0.041;
my $wheel_diameter_difference = 0.000000;	# usually between -0.000100 and +0.000100
my $wheel_diameter_left = $wheel_diameter - $wheel_diameter_difference;
my $wheel_diameter_right = $wheel_diameter +  $wheel_diameter_difference;

# Step 4
my $scaling_factor = 1;

# Calculate the effective axis length and wheel diameters
my $pi = 3.14159265358979;
my $distance_per_increment_left = $wheel_diameter_left * $scaling_factor * 2 * $pi / $increments_per_tour;
my $distance_per_increment_right = $wheel_diameter_right * $scaling_factor * 2 * $pi / $increments_per_tour;
my $axis_length = $axis_wheel_ratio * ($wheel_diameter_left + $wheel_diameter_right) / 2 * $scaling_factor;

# Write the configuration sentence
my $calibration_sentence = '$ODOMETRY,'.$axis_length.','.$distance_per_increment_left.','.$distance_per_increment_right;
print $calibration_sentence, "\n";
if ($robot_ip) {
	system('ssh', 'root@'.$robot_ip, 'echo "\\'.$calibration_sentence.'" > /etc/khepera/odometry');
}
