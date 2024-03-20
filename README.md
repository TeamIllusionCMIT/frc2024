# frc2024-main

team illusion frc code

## get started
this project uses poetry to manage dependencies. install it with `pip install poetry`.

to install the dependencies, run `poetry install`.

to enter the new virtual environment, use `poetry shell`.

## scripts
this project uses poe to manage scripts. to run a script, use `poe <script>`. to see all available scripts, use `poe`.
### check your code
```sh
poe precommit
```

### deploy the robot code
```sh
poe deploy
```