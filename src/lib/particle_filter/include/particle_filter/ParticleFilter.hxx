namespace particle_filter {

template <class StateType>
ParticleFilter<StateType>::ParticleFilter(unsigned int numParticles, std::shared_ptr<ObservationModel<StateType>> os,
                                          std::shared_ptr<MovementModel<StateType>> ms)
    : m_NumParticles(numParticles),
      m_ObservationModel(os),
      m_MovementModel(ms),
      m_FirstRun(true),
      m_ResamplingMode(RESAMPLE_NEFF) {
  m_ResamplingStrategy.reset(new ImportanceResampling<StateType>());

  assert(numParticles > 0);

  // allocate memory for particle lists
  m_CurrentList.resize(numParticles);
  m_LastList.resize(numParticles);

  double initialWeight = 1.0 / numParticles;
  // fill particle lists
  for (unsigned int i = 0; i < numParticles; i++) {
    m_CurrentList[i] = new Particle<StateType>(StateType(), initialWeight);
    m_LastList[i] = new Particle<StateType>(StateType(), initialWeight);
  }
}

template <class StateType>
ParticleFilter<StateType>::~ParticleFilter() {
  // release particles
  ConstParticleIterator iter;
  for (iter = m_CurrentList.begin(); iter != m_CurrentList.end(); ++iter) {
    delete *iter;
  }
  for (iter = m_LastList.begin(); iter != m_LastList.end(); ++iter) {
    delete *iter;
  }
}

template <class StateType>
unsigned int ParticleFilter<StateType>::numParticles() const {
  return m_NumParticles;
}

template <class StateType>
void ParticleFilter<StateType>::setObservationModel(std::shared_ptr<ObservationModel<StateType>> os) {
  m_ObservationModel = os;
}

template <class StateType>
std::shared_ptr<ObservationModel<StateType>> ParticleFilter<StateType>::getObservationModel() const {
  return m_ObservationModel;
}

template <class StateType>
void ParticleFilter<StateType>::setMovementModel(std::shared_ptr<MovementModel<StateType>> ms) {
  m_MovementModel = ms;
}

template <class StateType>
std::shared_ptr<MovementModel<StateType>> ParticleFilter<StateType>::getMovementModel() const {
  return m_MovementModel;
}

template <class StateType>
void ParticleFilter<StateType>::setResamplingStrategy(std::shared_ptr<ResamplingStrategy<StateType>> rs) {
  m_ResamplingStrategy = rs;
}

template <class StateType>
std::shared_ptr<ResamplingStrategy<StateType>> ParticleFilter<StateType>::getResamplingStrategy() const {
  return m_ResamplingStrategy;
}

template <class StateType>
void ParticleFilter<StateType>::setResamplingMode(ResamplingMode mode) {
  m_ResamplingMode = mode;
}

template <class StateType>
ResamplingMode ParticleFilter<StateType>::getResamplingMode() const {
  return m_ResamplingMode;
}

template <class StateType>
void ParticleFilter<StateType>::setPriorState(const StateType& priorState) {
  ConstParticleIterator iter;
  for (iter = m_CurrentList.begin(); iter != m_CurrentList.end(); ++iter) {
    (*iter)->setState(priorState);
  }
}

template <class StateType>
void ParticleFilter<StateType>::drawAllFromDistribution(
    const std::shared_ptr<StateDistribution<StateType>>& distribution) {
  ConstParticleIterator iter;
  for (iter = m_CurrentList.begin(); iter != m_CurrentList.end(); ++iter) {
    (*iter)->setState(distribution->draw());
  }
}

template <class StateType>
void ParticleFilter<StateType>::resetTimer() {
  m_FirstRun = true;
}

template <class StateType>
void ParticleFilter<StateType>::filter() {
  if (m_ResamplingMode == RESAMPLE_NEFF) {
    if (getNumEffectiveParticles() < m_NumParticles / 2) {
      resample();
    }
  } else if (m_ResamplingMode == RESAMPLE_ALWAYS) {
    resample();
  }  // else do not resample

  drift();
  diffuse();
  measure();
}

template <class StateType>
const Particle<StateType>* ParticleFilter<StateType>::getParticle(unsigned int particleNo) const {
  assert(particleNo < m_NumParticles);
  return m_CurrentList[particleNo];
}

template <class StateType>
const StateType& ParticleFilter<StateType>::getState(unsigned int particleNo) const {
  assert(particleNo < m_NumParticles);
  return m_CurrentList[particleNo]->getState();
}

template <class StateType>
double ParticleFilter<StateType>::getWeight(unsigned int particleNo) const {
  assert(particleNo < m_NumParticles);
  return m_CurrentList[particleNo]->getWeight();
}

template <class StateType>
void ParticleFilter<StateType>::sort() {
  std::sort(m_CurrentList.begin(), m_CurrentList.end(), CompareParticleWeights<StateType>());
}

template <class StateType>
void ParticleFilter<StateType>::resample() {
  // swap lists
  m_CurrentList.swap(m_LastList);
  // call resampling strategy
  m_ResamplingStrategy->resample(m_LastList, m_CurrentList);
}

template <class StateType>
void ParticleFilter<StateType>::drift(geometry_msgs::msg::Vector3 linear, geometry_msgs::msg::Vector3 angular) {
  for (unsigned int i = 0; i < m_NumParticles; i++) {
    m_MovementModel->drift(m_CurrentList[i]->m_State, linear, angular);
  }
}

template <class StateType>
void ParticleFilter<StateType>::diffuse() {
  // #pragma omp parallel for
  for (unsigned int i = 0; i < m_NumParticles; i++) {
    m_MovementModel->diffuse(m_CurrentList[i]->m_State);
  }
}

template <class StateType>
void ParticleFilter<StateType>::measure() {
  // measure only, if there are measurements available
  if (!m_ObservationModel->measurements_available()) {
    //    return;
    //    currently, this results in a problem, as the particle weight does
    //    not decay
  }
  double weight, weights_sum = 0;
  // #pragma omp parallel for
  for (size_t i = 0; i < m_NumParticles; i++) {
    // apply observation model

    // accumulate the weight when it is defined in the observation model
    if (m_ObservationModel->accumulate_weights_) {
      weight = m_CurrentList[i]->getWeightUnnormalized();
    } else {
      weight = 0;
    }

    // set explorer particle weight to minimal value if there are no
    // measurements available to reduce noise
    if (m_CurrentList[i]->is_explorer_ && !m_ObservationModel->measurements_available()) {
      weight += m_ObservationModel->get_min_weight();
    } else {
      weight += m_ObservationModel->measure(m_CurrentList[i]->getState());
    }
    m_CurrentList[i]->setWeight(weight);
    // Update the weight sum
    weights_sum += weight;
  }
  // #pragma omp parallel for
  for (size_t i = 0; i < m_NumParticles; i++) {
    m_CurrentList[i]->setNormalization(1.0 / weights_sum);
  }
  // re-sort the particles
  sort();
}

template <class StateType>
unsigned int ParticleFilter<StateType>::getNumEffectiveParticles() const {
  double squareSum = 0;
  for (unsigned int i = 0; i < m_NumParticles; i++) {
    double weight = m_CurrentList[i]->getWeight();
    squareSum += weight * weight;
  }
  return static_cast<int>(1.0f / squareSum);
}

template <class StateType>
const Particle<StateType>* ParticleFilter<StateType>::getBestParticle() const {
  return m_CurrentList[0];
}

template <class StateType>
double ParticleFilter<StateType>::getMaxParticleWeight() const {
  return m_CurrentList[0]->getWeight();
}

template <class StateType>
const StateType& ParticleFilter<StateType>::getBestState() const {
  return m_CurrentList[0]->getState();
}

template <class StateType>
StateType ParticleFilter<StateType>::getMmseEstimate() const {
  StateType estimate = m_CurrentList[0]->getState() * m_CurrentList[0]->getWeight();
  for (unsigned int i = 1; i < m_NumParticles; i++) {
    estimate += m_CurrentList[i]->getState() * m_CurrentList[i]->getWeight();
  }
  return estimate;
}

template <class StateType>
StateType ParticleFilter<StateType>::getBestXPercentEstimate(float percentage) const {
  StateType estimate = m_CurrentList[0]->getState() * m_CurrentList[0]->getWeight();
  double weightSum = m_CurrentList[0]->getWeight();
  unsigned int numToConsider = m_NumParticles / 100.0f * percentage;
  for (unsigned int i = 1; i < numToConsider; i++) {
    estimate += m_CurrentList[i]->getState() * m_CurrentList[i]->getWeight();
    weightSum += m_CurrentList[i]->getWeight();
  }
  estimate = estimate * (1.0 / weightSum);
  return estimate;
}

template <class StateType>
typename ParticleFilter<StateType>::ConstParticleIterator ParticleFilter<StateType>::particleListBegin() {
  return m_CurrentList.begin();
}

template <class StateType>
typename ParticleFilter<StateType>::ConstParticleIterator ParticleFilter<StateType>::particleListEnd() {
  return m_CurrentList.end();
}

template <class StateType>
visualization_msgs::msg::Marker ParticleFilter<StateType>::renderPointsMarker(std::string n_space, std::string frame,
                                                                              rclcpp::Duration lifetime,
                                                                              std_msgs::msg::ColorRGBA color) {
  return StateType::renderPointsMarker(m_CurrentList, n_space, frame, lifetime, color);
}

template <class StateType>
visualization_msgs::msg::MarkerArray ParticleFilter<StateType>::renderMarkerArray(std::string n_space,
                                                                                  std::string frame,
                                                                                  rclcpp::Duration lifetime,
                                                                                  std_msgs::msg::ColorRGBA color,
                                                                                  rclcpp::Time stamp) {
  visualization_msgs::msg::MarkerArray marker_array;

  for (unsigned int i = 0; i < m_NumParticles; i++) {
    double weight = m_CurrentList[i]->getWeight();
    color.r = weight / getMaxParticleWeight();
    marker_array.markers.push_back(m_CurrentList[i]->getState().renderMarker(n_space, frame, lifetime, color, stamp));
  }

  return marker_array;
}

template <class StateType>
gmms::GaussianMixtureModel ParticleFilter<StateType>::getGMM(int num_components, const double delta,
                                                             const int num_iterations, const bool ignore_explorers) {
  assert(num_components >= 1);
  gmms::GaussianMixtureModel gmm(num_components, delta, num_iterations);
  Eigen::MatrixXd dataset;
  StateType::convertParticleListToEigen(m_CurrentList, dataset, ignore_explorers);
  gmm.initialize(dataset);
  gmm.expectationMaximization(dataset);
  return gmm;
}

template <class StateType>
gmms::GaussianMixtureModel ParticleFilter<StateType>::getDynGMM(int min_num_components, int max_num_components,
                                                                const double component_delta,
                                                                const double iteration_delta, const int num_iterations,
                                                                const bool ignore_explorers) {
  assert(min_num_components < max_num_components);

  // set up dataset
  Eigen::MatrixXd dataset;
  StateType::convertParticleListToEigen(m_CurrentList, dataset, ignore_explorers);

  gmms::GaussianMixtureModel last_gmm, last_last_gmm;
  int component_count = 0;
  int component_number = min_num_components;
  double old_log_likelihood;
  double log_likelihood = 0.0;
  do {
    component_number = min_num_components + component_count;
    old_log_likelihood = log_likelihood;
    last_last_gmm = last_gmm;
    last_gmm = gmms::GaussianMixtureModel(component_number, iteration_delta, num_iterations);
    last_gmm.initialize(dataset);
    last_gmm.expectationMaximization(dataset);

    log_likelihood = last_gmm.logLikelihood(dataset);
    component_count++;
  } while (component_number < max_num_components and std::abs(old_log_likelihood - log_likelihood) > component_delta);

  if (std::abs(old_log_likelihood - log_likelihood) > component_delta or component_count <= 1) {
    return last_gmm;
  } else {
    return last_last_gmm;
  }
}

template <class StateType>
std::vector<std::vector<double>> ParticleFilter<StateType>::getCovarianceMatrix(const bool ignore_explorers) {
  // get an Eigen matrix of the particles
  Eigen::MatrixXd dataset;
  StateType::convertParticleListToEigen(m_CurrentList, dataset, ignore_explorers);
  Eigen::VectorXd component_means = dataset.colwise().mean();
  Eigen::VectorXd component_sums = dataset.colwise().sum();
  int component_num = dataset.cols();
  int particle_num = dataset.rows();
  Eigen::VectorXd variances = ((particle_num * component_means) - component_sums).cwiseAbs();
  std::vector<std::vector<double>> cov_mat(component_num, std::vector<double>(component_num, 0));
  for (int x = 0; x < component_num; x++) {
    for (int y = 0; y < component_num; y++) {
      cov_mat[x][y] = variances.coeff(x) * variances.coeff(y) / particle_num;
    }
  }
  return cov_mat;
}

template <class StateType>
std::vector<double> ParticleFilter<StateType>::getCovariance(float percentage) const {
  double xI = 0;
  double yI = 0;
  double thetaSinI = 0;
  double thetaCosI = 0;

  unsigned int numToConsider = m_NumParticles * (percentage / 100.0f);
  // std::cout << "number particles : " << m_NumParticles;
  // std::cout << "percentage: " << percentage;
  // std::cout << "nums to consider: " << numToConsider;
  unsigned int i = 0;
  for (; i < numToConsider; i++) {
    xI += std::round(m_CurrentList[i]->getState().getXPos() * 10000.0) / 10000.0;
    yI += std::round(m_CurrentList[i]->getState().getYPos() * 10000.0) / 10000.0;
    thetaSinI += std::round(m_CurrentList[i]->getState().getSinTheta() * 10000.0) / 10000.0;
    thetaCosI += std::round(m_CurrentList[i]->getState().getCosTheta() * 10000.0) / 10000.0;
  }

  // linear components
  StateType mean = getBestXPercentEstimate(percentage);

  double xMean = std::round(mean.getXPos() * 10000.0) / 10000.0;
  double yMean = std::round(mean.getYPos() * 10000.0) / 10000.0;

  double xIMean = xI - (numToConsider * xMean);
  double yIMean = yI - (numToConsider * yMean);

  xIMean = std::round(xIMean * 10000.0) / 10000.0;
  yIMean = std::round(yIMean * 10000.0) / 10000.0;
  double pos0 = (xIMean * xIMean) / numToConsider;
  double pos1 = (xIMean * yIMean) / numToConsider;
  double pos7 = (yIMean * yIMean) / numToConsider;

  // angular component
  // double s = thetaSinI;
  // double c = thetaCosI;

  thetaCosI = std::round(thetaCosI * 10000.0) / 10000.0;
  thetaSinI = std::round(thetaSinI * 10000.0) / 10000.0;

  double R = std::hypot(thetaCosI, thetaSinI);  // sqrt(c * c + s * s);
  R = std::round(R * 10000.0) / 10000.0;

  double Rmean = R / numToConsider;
  Rmean = std::round(Rmean * 10000.0) / 10000.0;

  // float pos36 = sqrt(-2 * log(sqrt(c * c + s * s))); // covariance not working and wrong
  // double pos36 = sqrt(-2 * log(Rmean));

  double pos35 = 1 - Rmean;  // variance

  pos0 = std::round(pos0 * 10000.0) / 10000.0;
  pos1 = std::round(pos1 * 10000.0) / 10000.0;
  pos7 = std::round(pos7 * 10000.0) / 10000.0;
  pos35 = std::round(pos35 * 10000.0) / 10000.0;

  /**

  if (pos36 < 0){
      std::cout << "nums to consider: " << numToConsider << std::endl;

      std::cout << "i: " << i << std::endl;
      std::cout << "theta sin I: " << thetaSinI << std::endl;
      std::cout << "theta cos I: " << thetaCosI << std::endl;
      std::cout << "pos36: " << pos36 << std::endl;


  }
   **/

  if (pos0 == -0.0) {
    pos0 = 0.0;
  }
  if (pos1 == -0.0) {
    pos1 = 0.0;
  }
  if (pos7 == -0.0) {
    pos7 = 0.0;
  }
  if (pos35 < 0.0) {
    pos35 = 0.0;
  } else if (pos35 > 1) {
    pos35 = 1.0;
  }

  std::vector<double> covariance = {pos0, pos1, 0, 0, 0, 0, pos1, pos7, 0, 0, 0, 0,  // pos6 == pos1
                                    0,    0,    0, 0, 0, 0, 0,    0,    0, 0, 0, 0,
                                    0,    0,    0, 0, 0, 0, 0,    0,    0, 0, 0, pos35};

  return covariance;
}

}  // namespace particle_filter
