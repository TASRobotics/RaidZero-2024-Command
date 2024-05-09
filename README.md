## Conventions:
 - Constants: All caps with underscores (eg: `SOME_CONSTANT_VALUE`)
 - Member variables: no "m" with camelCase (eg: `someMemberVariable`)
 - Subsystems have their own static instance in their own class
 - Class method order: Getters, setters, other methods (alphabetically)
 - Member def order: Grouped by types & alphabetical, but Talon configs then subsystem static object getter comes last
 - Subsystem constructors are private, will be called within class