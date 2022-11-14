# SpaceGrantEngine
'Engine' for handling concurrency and other 'low-level' tasks 

## Style Guide
Python code should follow snake case, with proper public/private seperation in class development.
Public/private in Python is done with an underscore at the front of an attribute. For example,

`self._example: str = "example"` is private
`self.example: str = "example"` is public

All Python code should also be type hinted. Type hints allow static checkers to analyze the 
codebase for any errors before runtime. This is done during the `make ci` command. Integrating
MyPy would enhance the static type checking further, but it is not done yet. Could be an item 
for someone to work on. (See the other workflow files in the .github folder)

## Using the Makefile
`make help`
Displays all possible commands for utilizing the Makefile.

The most used commands for typical work will be 
`make` or `make all` (they are equivalent)
`make ci`

`make` or `make all` both build the package and allow importing "sgengine" in Python.

`make ci` will run the continous integration and let you know if there are any linting errors.
These errors should be fixed before a pull request is opened to main, but are fine for the 
testing and development phase.

## Typical Development Workflow
1. Create a new branch
2. Switch to using the new branch as the active one
3. Create a subfolder with an "__init__.py" file 
4. Develop a class to add the given functionality OR a set of functions for the funcationality
5. Create a test file which uses fake input to test the code
6. Add the functionality to an existing node OR create a new node
7. 

## Common Github Commands
`git branch {branchname}` Creates a new local branch for development
`git checkout {branchname}` Switches your current branch to the given branch
`git add {files}` Adds files to git for tracking or adds updates
`git commit -m {message}` Makes a commit with the given files added with a given message
`git push` Pushes any local commits to the remote repository (Github) on your working branch
`git push --set-upstream origin {branchname}` Pushes changes to the given branchname remotely, if the branch was created locally
`git reset {--hard}` Resets back to the latest pushed changes in the remote repository (Github), DO NOT use if you have uncommited work.
