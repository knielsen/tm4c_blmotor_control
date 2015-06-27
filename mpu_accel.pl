#! /usr/bin/perl

use strict;
use warnings;

my $s;
my $i = 0;

while (1) {
  sysread(STDIN, $s, 6) or last;
  my $xi = (ord(substr($s, 0, 1)) << 8) | ord(substr($s, 1, 1));
  my $yi = (ord(substr($s, 2, 1)) << 8) | ord(substr($s, 3, 1));
  my $zi = (ord(substr($s, 4, 1)) << 8) | ord(substr($s, 5, 1));
  $xi -= 65536 if $xi >= 32768;
  $yi -= 65536 if $yi >= 32768;
  $zi -= 65536 if $zi >= 32768;
  my $x = $xi * (2/32768);
  my $y = $yi * (2/32768);
  my $z = $zi * (2/32768);
  print $i, "\t", $x, "\t", $y, "\t", $z, "\n";
  $i += 0.001;
}
