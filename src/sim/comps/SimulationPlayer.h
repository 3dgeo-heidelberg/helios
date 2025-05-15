#pragma once

class Simulation;
class Scene;
class ScenePart;
class Platform;
namespace helios {
namespace filems {
class FMSFacade;
}
}
using helios::filems::FMSFacade;
class Scanner;

#include <memory>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @brief Class handling how to play simulations.
 *
 * It does not handle the iterations inside the simulation loop itself but how
 * to handle with the simulation loop. The logic before and after a simulation
 * loop and also the decision on whether further simulation loops are required
 * belong to the SimulationPlayer.
 *
 * @see Simulation
 */
class SimulationPlayer
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The simulation that is being handled by the player.
   * @see Simulation
   */
  Simulation& sim;
  /**
   * @brief The scene of the simulation that is being handled by the player.
   * @see Scene
   */
  Scene& scene;
  /**
   * @brief The number of plays that have been run. It is initialized to
   *  zero and updated after each full simulation play.
   */
  int plays;
  /**
   * @brief The start point for the platform. Each consecutive replay
   *  will start with the platform at its start point.
   * @see SimulationPlayer::endPlay
   * @see SimulationPlayer::restartPlatform
   */
  std::shared_ptr<Platform> platformStart;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief SimulationPlayer constructor
   * @param sim The simulation to be handled.
   * @see Simulation
   * @see SimulationPlayer::sim
   */
  SimulationPlayer(Simulation& sim);
  virtual ~SimulationPlayer() = default;

  // ***  MAIN PUBLIC METHODS  *** //
  // ***************************** //
  /**
   * @brief Check whether there are pending plays.
   * @return True if there are pending plays, false otherwise.
   */
  bool hasPendingPlays();
  /**
   * @brief The logic after finishing a simulation loop.
   *
   * If further plays are needed, they will be typically prepared here.
   */
  void endPlay();
  /**
   * @biref The number of target plays, i.e., how many many times the
   *  simulation must be played.
   * @return How many times the simulation must be played.
   */
  int getNumTargetPlays();
  /**
   * @brief The number of computed plays, i.e., how many times the simulation
   *  has been played.
   * @return How many times the simulation has been played.
   */
  int getNumComputedPlays();
  /**
   * @brief The logic after finishing one play of the survey and before
   *  starting the next one.
   * @see SurveyPlayback
   * @see SurveyPlayback::startNextLeg
   * @see Simulation
   * @see Simulation::simPlayer
   */
  void prepareRepeat();

protected:
  // ***  UTIL PROTECTED METHODS  *** //
  // ******************************** //
  /**
   * @brief Restart a platform to its start point.
   * @param p The platform to be restarted.
   * @see SimulationPlayer::platformStart
   * @see Platform
   */
  void restartPlatform(Platform& p);
  /**
   * @brief Restart the file management system so it writes the replay's
   *  output to different files.
   * @param fms The file management system to be restarted.
   * @see FMSFacade
   */
  void restartFileMS(FMSFacade& fms);
  /**
   * @brief Restart a scanner to its start point.
   * @param sc The scanner to be restarted.
   * @see Scanner
   */
  void restartScanner(Scanner& sc);
  /**
   * @brief Restart a scene to its start point, considering the swapped
   *  geometries.
   * @param scene The scene to be restarted.
   * @param keepCRS Whether to keep the current scene's CRS (true) or not
   *  (false).
   * @see Scene
   * @see SwapOnRepeatHandler
   */
  void restartScene(Scene& scene, bool const keepCRS = true);
  /**
   * @brief Restart a simulation to its start point.
   * @param sim The simulation to be restarted.
   * @see Simulation
   * @see SimulationPlayer::sim
   */
  void restartSimulation(Simulation& sim);
  /**
   * @brief Check whether the current scene's CRS must be preserved (true) or
   *  not (false).
   *
   * The CRS will be preserved iff the keepCRS flag of each
   *  SwapOnRepeatHandler is set to True. It will be updated even if just
   *  one single handler has the keepCRS flag set to False.
   *
   * @param sorObjects The scene parts that have a swap on repeat handler.
   *  They can be obtained through the Scene::getSwapOnRepeatObjects
   *  method.
   * @return True if the current scene's CRS must be preserved, false
   *  otherwise.
   * @see SwapOnRepeatHandler
   * @see SwapOnRepeatHandler::keepCRS
   * @see SwapOnRepeatHandler::isKeepCRS
   * @see ScenePart
   * @see Scene
   * @see Scene::getSwapOnRepeatObjects
   */
  bool isKeepCRS(std::vector<std::shared_ptr<ScenePart>> const& sorObjects);
};
