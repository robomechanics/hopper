# Hopper
Organizing code for Hopper platform


### General Structure

Two separate components under development

##### Leg Actuation

Currently developing a single degree of freedom leg with spring in parallel to provide vertical actuation for the hopper. This leg has no role in body control as of now


Simscape model in place, basic optimization loop is functional. For low numbers of free variables this approach works reasonably well, but it looks like we'll need analytical equations of motion to run a full optimization loop in reasonable timescales.

*Tasks*

##### Inertial Control
Under development
