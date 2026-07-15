#!groovy

pipeline {

  libraries {
    lib('fe-pipeline-steps@1.6.0')
  }

  agent {
    node {
      label 'docker'
    }
  }

  triggers {
    pollSCM('H/5 * * * *')
  }

  environment {
    DEPENDENCIES_DIR = 'dependencies/'
    VERSION = feDetermineVersionFromGit()
    ARTIFACT_FILENAME = "robot-descriptions"
    DIST_DIR = "dist/usr/share/robot-descriptions"
  }

  options {
    // copyArtifactPermission('*')
    timeout(time: 2, unit: 'HOURS')
  }

  stages {
    // Stage to check commit message and skip pipeline if necessary
    stage('Check Commit Message') {
      steps {
        script {
          def commitMsg = sh(returnStdout: true, script: 'git log -1 --pretty=format:%s').trim()
          def skipKeywords = ['wip', '[skip ci]', '[skip-ci]', '[ci skip]', '[ci-skip]']
          if (skipKeywords.any { commitMsg.toLowerCase().contains(it) }) {
            currentBuild.result = 'ABORTED'
            error("Pipeline aborted because commit message contains skip directive.")
          }
        }
      }
    }

    stage('Get Ready') {
      steps {
        script {
          notifyBitbucket() // Notifies the Bitbucket instance of an INPROGRESS build
        }
        // Clean environment in case that previous build failed badly
        sh 'git clean -x -d -f'
      }
    }

    stage('Run URDF tests') {
      agent {
        dockerfile {
          dir '.docker'
          reuseNode true
        }
      }
      steps {
        sh '.ci/run_urdf_tests.sh'
      }
    }

    stage('Trigger upstream franka_ros2_ws builds') {
      parallel {
        stage('humble') {
          steps {
            build job: 'FCI/franka_ros2_ws/humble',
              wait: true,
              propagate: true,
              parameters: [string(name: 'frankaDescriptionBranch', value: env.BRANCH_NAME)]
          }
        }
        stage('jazzy') {
          steps {
            build job: 'FCI/franka_ros2_ws/jazzy',
              wait: true,
              propagate: true,
              parameters: [string(name: 'frankaDescriptionBranch', value: env.BRANCH_NAME)]
          }
        }
      }
    }

    stage('Build robot URDFs and SRDFs') {
      matrix {
        axes {
          axis {
            name 'ROBOT'
            values 'fp3', 'fr3', 'fr3v2', 'fr3v2_1', 'tmrv0_2'
          }
        }
        stages {
          stage('Build URDF and SRDF') {
            steps {
              sh "./scripts/create_urdf.sh --no-ee --no-prefix ${ROBOT}"
              dir ('urdfs') {
                stash name: "ROBOT_${ROBOT}_urdf", includes: "${ROBOT}.urdf"
                stash name: "ROBOT_${ROBOT}_srdf", includes: "${ROBOT}.srdf"
              }
            }
          }
        }
      }
    }

    stage('Build End Effector URDFs') {
      matrix {
        axes {
          axis {
            name 'EE'
            values 'franka_hand', 'cobot_pump'
          }
        }
        stages {
          stage('Build URDF') {
            steps {
              sh "./scripts/create_urdf.sh --only-ee --robot-ee ${EE}"
              dir ('urdfs') {
                stash name: "EE_${EE}_urdf", includes: "${EE}.urdf"
              }
            }
          }
        }
      }
    }

    stage('Archive URDFs and SRDFs') {
      agent {
        docker {
          image 'franka/build/ubuntu-24.04:0.27.0'
          reuseNode true
        }
      }

      stages {
        stage ('Cleanup') {
          steps {
            sh 'rm -rf usr'
          }
        }
        stage ('Create directory structure') {
          steps {
            sh "mkdir -p ${DIST_DIR}/meshes"
            sh "mkdir -p ${DIST_DIR}/robots"
            sh "mkdir -p ${DIST_DIR}/end_effectors"
          }
        }
        stage ('Move robot urdfs and srdfs into correct robot arm directory') {
          parallel {
            stage ('Arm3R') {
              steps {
                sh "mkdir -p ${DIST_DIR}/robots/Arm3R"
                dir("${DIST_DIR}/robots/Arm3R") {
                  unstash "ROBOT_fr3_urdf"
                  unstash "ROBOT_fr3_srdf"
                  sh "mv fr3.urdf robot.urdf"
                  sh "mv fr3.srdf robot.srdf"
                }
              }
            }
            stage ('Arm3P') {
              steps {
                sh "mkdir -p ${DIST_DIR}/robots/Arm3P"
                dir("${DIST_DIR}/robots/Arm3P") {
                  unstash "ROBOT_fp3_urdf"
                  unstash "ROBOT_fp3_srdf"
                  sh "mv fp3.urdf robot.urdf"
                  sh "mv fp3.srdf robot.srdf"
                }
              }
            }
            stage ('Arm3Rv2') {
              steps {
                sh "mkdir -p ${DIST_DIR}/robots/Arm3Rv2"
                dir ("${DIST_DIR}/robots/Arm3Rv2") {
                  unstash "ROBOT_fr3v2_urdf"
                  unstash "ROBOT_fr3v2_srdf"
                  sh "mv fr3v2.urdf robot.urdf"
                  sh "mv fr3v2.srdf robot.srdf"
                }
              }
            }
            stage ('Arm3Rv2.1') {
              steps {
                sh "mkdir -p ${DIST_DIR}/robots/Arm3Rv2.1"
                dir ("${DIST_DIR}/robots/Arm3Rv2.1") {
                  unstash "ROBOT_fr3v2_1_urdf"
                  unstash "ROBOT_fr3v2_1_srdf"
                  sh "mv fr3v2_1.urdf robot.urdf"
                  sh "mv fr3v2_1.srdf robot.srdf"
                }
              }
            }
            stage ('TMRv0.2') {
              steps {
                sh "mkdir -p ${DIST_DIR}/robots/TMRv0.2"
                dir ("${DIST_DIR}/robots/TMRv0.2") {
                  unstash "ROBOT_tmrv0_2_urdf"
                  unstash "ROBOT_tmrv0_2_srdf"
                  sh "mv tmrv0_2.urdf robot.urdf"
                  sh "mv tmrv0_2.srdf robot.srdf"
                }
              }
            }
          }
        }
        stage ('Copy meshes to dist') {
          steps {
            script {
              sh "mkdir -p ${DIST_DIR}/meshes/robots"
              def robs = ['fp3', 'fr3', 'fr3v2', 'fr3v2_1', 'tmrv0_2']
              def copyCmd = robs.collect { "cp -r meshes/robots/${it} ${DIST_DIR}/meshes/robots" }.join(' && ')
              sh copyCmd
            }
          }
        }
        stage ('Extract End Effector URDFs') {
            stages {
              stage('Create End Effector directories') {
                steps {
                  sh "mkdir -p ${DIST_DIR}/end_effectors/cobot_pump"
                  sh "mkdir -p ${DIST_DIR}/end_effectors/franka_hand"
                }
              }
              stage ('Copy meshes') {
                parallel {
                  stage ('cobot_pump') {
                    steps {
                      sh '''
                        mkdir -p "${DIST_DIR}/meshes/robot_ee/cobot_pump"
                        cp -r meshes/robot_ee/cobot_pump/* "${DIST_DIR}/meshes/robot_ee/cobot_pump"
                      '''
                    }
                  }
                  stage ('franka_hand') {
                    steps {
                      sh '''
                        mkdir -p "${DIST_DIR}/meshes/robot_ee/franka_hand_white"
                        cp -r meshes/robot_ee/franka_hand_white/* "${DIST_DIR}/meshes/robot_ee/franka_hand_white"
                      '''
                    }
                  }
                }
              }
              stage('Unstash URDF') {
                steps {
                  dir("${DIST_DIR}/end_effectors/cobot_pump") {
                    unstash "EE_cobot_pump_urdf"
                    sh "mv cobot_pump.urdf endeffector.urdf"
                  }
                  dir("${DIST_DIR}/end_effectors/franka_hand") {
                    unstash "EE_franka_hand_urdf"
                    sh "mv franka_hand.urdf endeffector.urdf"
                  }
                }
              }
            }
        }
        stage ('Create archive') {
          steps {
            sh "rm -rf ${DIST_DIR}/robots/*@tmp"
            sh "rm -rf ${DIST_DIR}/end_effectors/*@tmp"

            dir ('dist'){
              sh '''
                tar czvf "${ARTIFACT_FILENAME}_${VERSION}.tar.gz" usr/share
                fpm -s tar \
                    -t deb \
                    -n "robot-descriptions" \
                    -v ${VERSION} \
                    -a all \
                    --vendor "Franka Robotics GmbH" \
                    --maintainer "Franka Robotics GmbH <support@franka.de>" \
                    "${ARTIFACT_FILENAME}_${VERSION}.tar.gz"
                # Rename .deb to .ipk for opkg-compatible publishing via fePublishOpkg
                mv "${ARTIFACT_FILENAME}_${VERSION}_all.deb" "${ARTIFACT_FILENAME}_${VERSION}_all.ipk"
                printf "%s archive created.\nversion: %s\nsha1sum: %s\n" "${ARTIFACT_FILENAME}" "${VERSION}" "$(sha1sum "${ARTIFACT_FILENAME}_${VERSION}_all.ipk")"
              '''
              archiveArtifacts '*.tar.gz'
            }
          }
        }
        stage ('Publish to artifactory') {
          when {
            not {
              equals expected: 'UNSTABLE', actual: currentBuild.result
            }
            anyOf {
              buildingTag()
              branch "main"  // main should always be published, it is the fallback for other pipelines
              // branch "${env.BRANCH_NAME}" // or current branch? 
            }
          }
          steps {
            fePublishOpkg("*.ipk", "mc7-5x")
          }
        }
      }
    }
  } // stages

  // post is guaranteed to run at the end of a Pipeline’s execution
  post {
    success {
      fePublishBuildInfo()
    }
    always {
      sh 'git clean -x -d -f'
      cleanWs()
      sh 'du -h --max-depth=1 . && ls -la'
      script {
        notifyBitbucket()
      }
    }
  }// post
}// pipeline
