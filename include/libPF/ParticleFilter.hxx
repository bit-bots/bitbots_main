namespace libPF
{


template <class StateType>
ParticleFilter<StateType>::ParticleFilter(unsigned int numParticles, ObservationModel<StateType>* os, MovementModel<StateType>* ms) :
    m_NumParticles(numParticles),
    m_ObservationModel(os),
    m_MovementModel(ms),
    m_ResamplingStrategy(&m_DefaultResamplingStrategy),
    m_FirstRun(true),
    m_ResamplingMode(RESAMPLE_NEFF)
{

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
    for (iter = m_CurrentList.begin(); iter != m_CurrentList.end(); ++iter)
    {
        delete *iter;
    }
    for (iter = m_LastList.begin(); iter != m_LastList.end(); ++iter)
    {
        delete *iter;
    }
}


template <class StateType>
unsigned int ParticleFilter<StateType>::numParticles() const {
  return m_NumParticles;
}

template <class StateType>
void ParticleFilter<StateType>::setObservationModel(ObservationModel<StateType>* os) {
    m_ObservationModel = os;
}

template <class StateType>
ObservationModel<StateType>* ParticleFilter<StateType>::getObservationModel() const {
    return m_ObservationModel;
}

template <class StateType>
void ParticleFilter<StateType>::setMovementModel(MovementModel<StateType>* ms) {
    m_MovementModel = ms;
}

template <class StateType>
MovementModel<StateType>* ParticleFilter<StateType>::getMovementModel() const {
    return m_MovementModel;
}

template <class StateType>
void ParticleFilter<StateType>::setResamplingStrategy(ResamplingStrategy<StateType>* rs) {
    m_ResamplingStrategy = rs;
}

template <class StateType>
ResamplingStrategy<StateType>* ParticleFilter<StateType>::getResamplingStrategy() const {
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
    for (iter = m_CurrentList.begin(); iter != m_CurrentList.end(); ++iter)
    {
        (*iter)->setState(priorState);
    }
}

template <class StateType>
void ParticleFilter<StateType>::drawAllFromDistribution(const StateDistribution<StateType>& distribution) {
    ConstParticleIterator iter;
    for (iter = m_CurrentList.begin(); iter != m_CurrentList.end(); ++iter)
    {
        (*iter)->setState(distribution.draw());
    }
}

template <class StateType>
void ParticleFilter<StateType>::resetTimer() {
    m_FirstRun = true;
}

template <class StateType>
void ParticleFilter<StateType>::filter(double dt) {
    if (m_ResamplingMode == RESAMPLE_NEFF) {
        if (getNumEffectiveParticles() < m_NumParticles / 2) {
            resample();
        }
    } else if (m_ResamplingMode == RESAMPLE_ALWAYS) {
        resample();
    } // else do not resample

    if (dt < 0.0) // use internal time measurement
    {
        // for the first run, we have no information about the time interval
        if (m_FirstRun) {
            m_FirstRun = false;
            m_LastDriftTime = clock();
        }
        clock_t currentTime = clock();
        dt = ((double)currentTime - (double)m_LastDriftTime) / CLOCKS_PER_SEC;
        m_LastDriftTime = currentTime;
    }
    drift(dt);
    diffuse(dt);
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
void ParticleFilter<StateType>::normalize() {
    double weightSum = 0.0;
    ConstParticleIterator iter;
    for (iter = m_CurrentList.begin(); iter != m_CurrentList.end(); ++iter) {
        weightSum += (*iter)->getWeight();
    }
    // only normalize if weightSum is big enough to devide
    if (weightSum > m_NumParticles * std::numeric_limits<double>::epsilon()) {
        double factor = 1.0 / weightSum;
        for (iter = m_CurrentList.begin(); iter != m_CurrentList.end(); ++iter) {
            double newWeight = (*iter)->getWeight() * factor;
            (*iter)->setWeight(newWeight);
        }
    } else {
        std::cerr << "WARNING: ParticleFilter::normalize(): Particle weights *very* small!" << std::endl;
    }
}

template <class StateType>
void ParticleFilter<StateType>::resample() {
  // swap lists
  m_CurrentList.swap(m_LastList);
  // call resampling strategy
  m_ResamplingStrategy->resample(m_LastList, m_CurrentList);
}


template <class StateType>
void ParticleFilter<StateType>::drift(double dt) {
  for (unsigned int i = 0; i < m_NumParticles; i++) {
    m_MovementModel->drift(m_CurrentList[i]->m_State, dt);
  }
}

template <class StateType>
void ParticleFilter<StateType>::diffuse(double dt) {
  for (unsigned int i = 0; i < m_NumParticles; i++) {
    m_MovementModel->diffuse(m_CurrentList[i]->m_State, dt);
  }
}

template <class StateType>
void ParticleFilter<StateType>::measure() {
  for (unsigned int i = 0; i < m_NumParticles; i++) {
    // apply observation model
    double weight = m_ObservationModel->measure(m_CurrentList[i]->getState());
    m_CurrentList[i]->setWeight(weight);
  }
  // after measurement we have to re-sort and normalize the particles
  sort();
  normalize();
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
typename ParticleFilter<StateType>::ConstParticleIterator ParticleFilter<StateType>::particleListBegin()
{
    return m_CurrentList.begin();
}

template <class StateType>
typename ParticleFilter<StateType>::ConstParticleIterator ParticleFilter<StateType>::particleListEnd()
{
    return m_CurrentList.end();
}

} // end of namespace
