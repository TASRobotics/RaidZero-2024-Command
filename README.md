## Conventions:
 - Constants: All caps with underscores (eg: `SOME_CONSTANT_VALUE`)
 - Member variables: no "m" with camelCase (eg: `someMemberVariable` instead of `mSomeMemberVariable`)
 - Subsystems should be in the form of a singleton class
 - Class method order: Getters, setters, other methods (alphabetically)
 - Member def order: Grouped logically..., but Talon configs then subsystem static object getter comes last
   - Sample order:
     1. Some Motor
     2. Some sensor
     3. Voltage/DutyCycle configs (If applicable)
     4. Static subsystem instance
     5. Private class constructor
     6. Getters
     7. Setters
     8. Other methods (Orderd based on functionality)
     9. Subsystem instance getter
     10. Subsystem configs (Eg, Swerve will have AutoBuilder config)
     11. Device configs (If applicable)
 - Subsystem constructors are private and will be called within the class
 - Import statements: let Java RedHat extension take care of it
 - Spaces in code where necessary (Eg, between methods, between operations such as + or -)
 - Braces are on the same line as method declarations
 - Always add Javadoc to methods!!
 - Branches:
   - Base branch should contain subsystem name (Eg, `arm` or `swerve`)
   - Child branches should:
     - Be appended to parent branch's name with a `/`
     - Describe the feature being worked on (Eg, `arm/pid` or `swerve/vision-odometry`)
     - Should not be long!
