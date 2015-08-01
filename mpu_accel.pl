#! /usr/bin/perl

use strict;
use warnings;

my $s;
my $i = 0;

my ($mx, $my, $mz) = 0;
my ($sx, $sy, $sz) = 0;
my $n = 0;

my $d = [ ];

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

  $mx += $x;
  $my += $y;
  $mz += $z;
  ++$n;
  push @$d, [$x, $y, $z];
}

$mx /= $n;
$my /= $n;
$mz /= $n;
print STDERR "Mean:\t", $mx, "\t", $my, "\t", $mz, "\n";

for (@$d) {
  my $tmp = $_->[0] - $mx;
  $sx += $tmp*$tmp;
  $tmp = $_->[1] - $my;
  $sy += $tmp*$tmp;
  $tmp = $_->[2] - $mz;
  $sz += $tmp*$tmp;
}
$sx /= ($n-1);
$sy /= ($n-1);
$sz /= ($n-1);
print STDERR "Stddev:\t", sqrt($sx), "\t", sqrt($sy), "\t", sqrt($sz), "\n";
