Before running add a game_controller.yaml in the config folder, with the following content:

```yaml
humanoid_league_game_controller:
  ros__parameters:
    team_id: YOUR_TEAM_ID
    bot_id: NUMBER_OF_THIS_ROBOT
```

Then launch with:
    `ros2 lauch humanoid_league_game_controller game_controller.launch`
