function out=orientation_estimation_ahrs_fun(acc,gyro,mag,fs)

%% construct error state Kalman function
  dT=1/fs;
  cOrientErrVar    = deg2rad(1)*deg2rad(1)*2000e-5; % var in init orientation error estim.
  cGyroBiasErrVar  = deg2rad(1)*deg2rad(1)*250e-3*0.1; % var in init gyro bias error estim
  cOrientGyroBiasErrVar = deg2rad(1)*deg2rad(1)*0; % covar orient -gyro bias error estim
  cAccErrVar       = 10e-5 * (9.81^2);  % var in linear accel drift error estim
  % set Q
  theta_w=cOrientErrVar*ones(1,3);
  b_w=cGyroBiasErrVar*ones(1,3);
  a_w=cAccErrVar*ones(1,3);
  m_w=0.6*ones(1,3);
  w=[theta_w,b_w,a_w,m_w];
  Qw=diag(w);
  % set R
  k2=dT*dT;
  % gyroscopeDriftNoise
  beta=3.0462*10^(-13);
  % GyroscopeNoise
  yita =9.1385*10^(-5);
  %  AccelerometerNoise
  lamda=0.00019247;
  % MagnetometerNoise 
  lamda_m=0.1;
  % LinearAccelerationNoise
  xi=0.0096236;
  % MagneticDisturbanceNoise gamma
  xi_m=0.3;
  %LinearAcclerationDecayFactor 
  nv=0.3;
  % MagneticDisturbanceDecayFactor  sigama
  nv_m=0.3;
  
  % ExpectedMagneticFieldStrength 
  MagSth=50;
  pMagVec=[MagSth 0 0];
  
  Ra= (lamda+xi+k2*(beta+yita))*eye(3);
  Rw= (lamda_m+xi_m+k2*(beta+yita))*eye(3);
  R=[Ra,0*eye(3);
     0*eye(3), Rw];
  % set GyroOffset
  gyro_offset=zeros(1,3);
  % set linear acc prior
  linAccelPrior=zeros(1,3);
  linAccelPost=zeros(1,3);
  % magnetic disturbance
  magDisPrior=zeros(1,3);
  magDisPost=zeros(1,3);
  % set initial P_
  P_=Qw;
  
  len=length(acc);
  
for i=1:len
   accr=acc(i,:);
   gyror=gyro(i,:);
   magr=mag(i,:);
   if(i==1)
   % NED coordinate
   r_down=accr';
   r_east=cross(accr',magr');
   r_north= cross(r_east, r_down);
   
   r_down=r_down/norm(r_down);
   r_east=r_east/norm(r_east);
   r_north=r_north/norm(r_north);
   
   R_=[r_north,r_east,r_down];
   Q__ = quaternion(R_, 'rotmat', 'frame');
   end
   %% without magnetic value
   % predict
   Q_delta=(gyror(1:3)-gyro_offset)*dT;
   Q_delta=quaternion(Q_delta,'rotvec');
   Q_=Q__*Q_delta;
   if parts(Q_) < 0
      Q_ = -Q_;
   end
   %compare with ecompass command
   Rprior = rotmat(Q_, 'frame'); 
   % The Kalman filter measurement:
   linAccelPrior=nv*linAccelPost;
   % estimate gravity from acceleration
   gAccel=accr+linAccelPrior;
   g=Rprior(:,3)'*9.81;
   gdiffer=gAccel-g;
   % Earth's mag vec (uT) from gyro updates
   magVecGyroMeas=(Rprior*pMagVec')';
   % estimate of disturbance
   k_mag=nv_m;
   %k_mag=0;
   magDisPrior=k_mag*magDisPost;
   magr_prior=magr-magDisPrior;
   mdiffer=magr_prior-magVecGyroMeas;
   %mdiffer=[0,0,0];
   m=magVecGyroMeas;
   % observer model
   k=dT;
   H=[0 g(3) -g(2) 0 -k*g(3) k*g(2) 1 0 0 0 0 0;
      -g(3) 0 g(1) k*g(3) 0 -k*g(1) 0 1 0 0 0 0;
      g(2) -g(1) 0 -k*g(2) k*g(1) 0 0 0 1 0 0 0;
      0 m(3) -m(2) 0 -k*m(3) k*m(2) 0 0 0 -1 0 0;
      -m(3) 0 m(1)  k*m(3) 0 -k*m(1) 0 0 0  0 -1 0;
      m(2) -m(1) 0 -k*m(2) k*m(1) 0 0 0 0 0 0 -1];

  % innovation convariance
  S=R+H*(P_)*H';
  % Kalman Gain
  K=(P_)*H'/S';
  % Updata a Posterior Error
  ze=[gdiffer';mdiffer'];
  magDistErr=(K(10:12,:)*ze)';
  magDistPower = (norm(magDistErr.')).^2;
  isJamming=(magDistPower>(4*MagSth^2));
  if(isJamming)
    jamze=  gdiffer';
    jamxe_post = K(1:9, 1:3) * jamze;
    orientErr = jamxe_post(1:3).';
    gyroOffsetErr = jamxe_post(4:6).';
    linAccelErr = jamxe_post(7:9).';   
  else
     xe_post=K*ze; 
     % Error parts of xe_post
     orientErr = xe_post(1:3).';
     gyroOffsetErr = xe_post(4:6).';
     linAccelErr = xe_post(7:9).';
  end

  % update posterior estimate
  qerr = conj(quaternion(orientErr, 'rotvec'));
  Q__=Q_*qerr;
  % Force rotation angle to be positive
  if parts(Q__) < 0
    Q__ = -Q__;
  end
  Q__=normalize(Q__);
  Rpost = rotmat(Q__, 'frame');
  % estimate linear acceleartion
  linAccelPost=linAccelPrior-linAccelErr;
  % estimate Gyroscope Offset
  gyro_offset=gyro_offset-gyroOffsetErr;

  magDisPost=magDisPrior-magDistErr;
  
  % If no jamming, update the Magnetic Vector estimate 
  if ~isJamming
     magDistErrGlobal = ((Rpost.')*magDistErr.').'; 
     mtmp = pMagVec- magDistErrGlobal;
     inclination = atan2(mtmp(3),mtmp(1));
     if inclination < -pi/2
          inclination(:) = -pi/2;
     end
     if inclination > pi/2
         inclination(:) = pi/2;
     end
     % Use the inclination angle to build a pMagVec
     pMagVec=zeros(1,3,'double');
     pMagVec(1)=cos(inclination);
     pMagVec(3)=sin(inclination);
     pMagVec=MagSth*pMagVec;
     
%      pMagVec'
%      pMagVec1=Rpost*(magr_prior+magDistErr)';
%      pMagVec1(1)=norm([pMagVec1(1),pMagVec1(2)]);
%      pMagVec1(2)=0;
%      50*pMagVec1./norm(pMagVec1)
     
  end
  
  P__=P_-K*(H*P_);
  
  % predict error estimate covariance for next iteration
  k=dT;
  Qw=[P__(1)+k^2*(P__(40)+beta+yita) 0 0 -k*(P__(40)+beta) 0 0 0 0 0 0 0 0;
     0 P__(14)+k^2*(P__(53)+beta+yita) 0 0 -k*(P__(53)+beta) 0 0 0 0 0 0 0;
     0 0 P__(27)+k^2*(P__(66)+beta+yita) 0 0 -k*(P__(66)+beta) 0 0 0 0 0 0;
     -k*(P__(40)+beta) 0 0 P__(40)+beta 0 0 0 0 0 0 0 0;
     0 -k*(P__(53)+beta) 0 0 P__(53)+beta 0 0 0 0 0 0 0;
     0 0 -k*(P__(66)+beta) 0 0 P__(66)+beta 0 0 0 0 0 0;
     0 0 0 0 0 0 nv^2*P__(79)+xi 0 0 0 0 0;
     0 0 0 0 0 0 0 nv^2*P__(92)+xi 0 0 0 0;
     0 0 0 0 0 0 0 0  nv^2*P__(105)+xi 0 0 0;
     0 0 0 0 0 0 0 0 0 nv_m^2*P__(118)+xi_m 0 0;
     0 0 0 0 0 0 0 0 0 0 nv_m^2*P__(131)+xi_m 0;
     0 0 0 0 0 0 0 0 0 0 0 nv_m^2*P__(144)+xi_m];
    % Update error covariance matrix 
    P_=Qw;
    % compute Augular Velocity
    %augVel(i,:)=gyror-gyro_offset;
    %output
    Quat(i,:)=Q__; 
    acc(i,:)=linAccelPost;
    offset(i,:)=gyro_offset;
    mag_d(i,:)=magDisPost;
    MagPower(i)=magDistPower;
    % change quaternion to eular 
   
end

elu=eulerd(Quat,'YXZ','frame');
out.Quat=Quat;
out.elu=elu;
out.acc=acc;
out.mag_d=mag_d;
out.MagPower=MagPower;
out.offset=offset;


end