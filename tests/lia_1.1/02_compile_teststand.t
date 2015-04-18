#!/usr/bin/perl -w

use Test::More tests => 1;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use Program;
use Proc::Background;

$|++; 

####################
# Make the airframe
my $make_compile_options = "AIRCRAFT=Quad_Lia_ovgu_teststand_01 clean_ac ap.compile";
my $compile_output = run_program(
	"Attempting to build the firmware.",
	$ENV{'PAPARAZZI_SRC'},
	"make $make_compile_options",
	0,1);
unlike($compile_output, '/\bError\b/i', "The compile output does not contain the word \"Error\"");


################################################################################
# functions used by this test script.
sub run_program
{
        my $message = shift;
        my $dir = shift;
        my $command = shift;
        my $verbose = shift;
        my $dont_fail_on_error = shift;

        warn "$message\n" if $verbose;
        if (defined $dir)
        {
                $command = "cd $dir;" . $command;
        }
        my $prog = new Program("bash");
        my $fh = $prog->open("-c \"$command\"");
	warn "Running command: \"". $prog->last_command() ."\"\n" if $verbose;
        $fh->autoflush(1);
        my @output;
        while (<$fh>)
        {
		warn $_ if $verbose;
		chomp $_;
                push @output, $_;
        }
        $fh->close;
        my $exit_status = $?/256;
        unless ($exit_status == 0)
        {
                if ($dont_fail_on_error)
                {
                        warn "Error: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n" if $verbose;
                }
                else
                {
                        die "Error: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n";
                }
        }
        return wantarray ? @output : join "\n", @output;
}

