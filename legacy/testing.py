import unittest
import numpy as np
from types import SimpleNamespace
from solver import solve, computeNumberOfPoints, computeAzimuthAndOrigin, computeCollocationPoints, computeChords, computeVrotational, computeVaxial, computeVtangential, computeInflowAndAlpha, computeForces  # import only what's needed
from geometry import defineDrone  # import defineDrone directly


class TestSolver(unittest.TestCase):
    
    def setUp(self):
        filename = 'mock.json'  
        self.drone = defineDrone(filename, main_U=None, small_U=None, main_RPM=None, small_RPM=None)

    def test_main_propeller_count(self):
        n = self.drone.main_prop.n
        self.assertEqual(n, 4, "Main propeller n should be 4")  # Adjusted message to match expected value

    def test_computeNumberOfPoints(self):
        npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(self.drone)
        self.assertEqual(main_n, 4, "Number of main points should be 4")
        self.assertEqual(small_n, 5, "Number of small points should be 5")
        self.assertEqual(npM, 9, "Number of main propeller collocs should be 9")
        self.assertEqual(npS, 12, "Number of small propeller collocs should be 12")
        self.assertEqual(collocN, 36+9, "Number of collocation points should be 36+9=45")
        self.assertEqual(main_NB, 3, "Number of main propeller boundary points should be 3")
        self.assertEqual(small_NB, 3, "Number of small propeller boundary points should be 3")

    def test_computeAzimuthAndOrigin(self):
        npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(self.drone)
        n_azimuth, n_origin = computeAzimuthAndOrigin(self.drone, npM, npS, main_NB, collocN)
        np.testing.assert_array_equal(
            n_azimuth[:npM],
            np.array([[0, 0, 1]] * npM),
            err_msg="n_azimuth main should be [[0,0,1]] repeated npM times"
        )

        np.testing.assert_allclose(
            n_azimuth[npM: npM + npS],  # small propeller section
            np.array([[-1, 0, 0]] * npS),
            rtol=0,
            atol=1e-8,
            err_msg="n_azimuth small should be [[-1, 0, 0]] repeated npS times"
        )

        np.testing.assert_allclose(
            n_azimuth[npM + npS:npM+2*npS],
            np.array([[np.cos(60*np.pi/180), -np.sin(60*np.pi/180), 0]] * npS),
            rtol=0,
            atol=1e-8,
            err_msg="n_azimuth small should be [[cos(60), -sin(60),0]] repeated npS times"
        )

        np.testing.assert_allclose(
            n_azimuth[npM + 2*npS:],
            np.array([[np.cos(60*np.pi/180), np.sin(60*np.pi/180), 0]] * npS),
            rtol=0,
            atol=1e-8,
            err_msg="n_azimuth small should be [[cos(30), -sin(30),0]] repeated npS times"
        )

        print('n_azimuth computed correctly')

        #________________________

        np.testing.assert_allclose(
            n_origin[:npM],
            np.array([[0, 0, 0]] * npM),
            rtol=0,
            atol=1e-8,
            err_msg="n_origin main should be [[0,0,0]] repeated npM times"
        )

        np.testing.assert_allclose(
            n_origin[npM: npM + npS],
            np.array([[0, 1, 0]] * npS),
            rtol=0,
            atol=1e-8,
            err_msg="n_origin small should be [[0,0,0]] repeated npS times"
        )

        np.testing.assert_allclose(
            n_origin[npM + npS:npM + 2*npS],
            np.array([[-np.cos(30*np.pi/180), - np.sin(30*np.pi/180), 0]] * npS),
            rtol=0,
            atol=1e-8,
            err_msg="n_origin small should be [-cos(30), -sin(30), 0] repeated npS times"
        )

        np.testing.assert_allclose(
            n_origin[npM + 2*npS:],
            np.array([[np.cos(30*np.pi/180), - np.sin(30*np.pi/180), 0]] * npS),
            rtol=0,
            atol=1e-8,
            err_msg="n_origin small should be [cos(30), -sin(30), 0] repeated npS times"
        )

        print('n_origin computed correctly')


    def test_computeCollocationPoints(self):
        npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(self.drone)
        collocs = computeCollocationPoints(self.drone, npM, npS, main_NB, small_NB, main_n, small_n)
        np.testing.assert_allclose(
            collocs[:npM, :],
            np.array([[0,0.25, 0],
                      [0,0.55, 0],
                      [0,0.85, 0],

                      [-0.25*np.cos(30*np.pi/180), - 0.25*np.sin(30*np.pi/180), 0],
                      [-0.55*np.cos(30*np.pi/180), - 0.55*np.sin(30*np.pi/180), 0],
                      [-0.85*np.cos(30*np.pi/180), - 0.85*np.sin(30*np.pi/180), 0],

                      [0.25*np.cos(30*np.pi/180), - 0.25*np.sin(30*np.pi/180), 0],
                      [0.55*np.cos(30*np.pi/180), - 0.55*np.sin(30*np.pi/180), 0],
                      [0.85*np.cos(30*np.pi/180), - 0.85*np.sin(30*np.pi/180), 0],


                      ]),
            rtol=0,
            atol=1e-8,
        )

        np.testing.assert_allclose(
            collocs[npM:npM+4, :],
            np.array([[0, 1.03, 0],
                      [0, 1.05, 0], 
                      [0, 1.07, 0], 
                      [0, 1.09, 0]

                      ]),
            rtol=0,
            atol=1e-8,
        )

        print('collocation points computed correctly')

    def test_computeChords(self):
        npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(self.drone)
        chords = computeChords(self.drone, npM, collocN, main_NB, small_NB, main_n, small_n)
        
        np.testing.assert_allclose(
            chords[:npM, :],
            np.ones((npM,1))*0.1  ,
            rtol=0,
            atol=1e-8,
        )
        np.testing.assert_allclose(
            chords[npM:, :],
            np.ones((npS*3,1))*0.01,
            rtol=0,
            atol=1e-8,
        )
        print('chords computed correctly')

    def test_computeVrotational(self):
        npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(self.drone)
        total_colloc_points = computeCollocationPoints(self.drone, npM, npS, main_NB, small_NB, main_n, small_n)
        n_azimuth, n_origin = computeAzimuthAndOrigin(self.drone, npM, npS, main_NB, collocN)

        Omega = np.zeros((collocN, 1))
        Omega[:npM] = self.drone.main_prop.RPM*2*np.pi/60
        Omega[npM:] = self.drone.small_props[0].RPM*2*np.pi/60
        np.testing.assert_allclose(
            Omega[:npM],
            np.ones((npM,1))*self.drone.main_prop.RPM*2*np.pi/60,
            rtol=0,
            atol=1e-8,
        )
        np.testing.assert_allclose(
            Omega[npM:],
            np.ones((npS*3,1))*self.drone.small_props[0].RPM*2*np.pi/60,
            rtol=0,
            atol=1e-8,
        )
        print('Omega computed correctly')   

        v_rotational, v_rotational_main, v_rotational_small = computeVrotational(self.drone, total_colloc_points, Omega, n_azimuth, n_origin, npM, npS, main_NB)
        np.testing.assert_allclose(
            v_rotational[:npM, :],
            np.array([[10.4719755, 0, 0],
                      [23.0383461, 0, 0],
                      [35.6047167, 0, 0],

                      [-10.4719755*np.sin(30*np.pi/180), 10.4719755*np.cos(30*np.pi/180), 0],
                      [-23.0383461*np.sin(30*np.pi/180), 23.0383461*np.cos(30*np.pi/180), 0],
                      [-35.6047167*np.sin(30*np.pi/180), 35.6047167*np.cos(30*np.pi/180), 0],

                      [-10.4719755*np.sin(30*np.pi/180), -10.4719755*np.cos(30*np.pi/180), 0],
                      [-23.0383461*np.sin(30*np.pi/180), -23.0383461*np.cos(30*np.pi/180), 0],
                      [-35.6047167*np.sin(30*np.pi/180), -35.6047167*np.cos(30*np.pi/180), 0],
                      
                      ]),
            rtol=0,
            atol=1e-6,
        )

        print('v_rotational_small', v_rotational_small)

        print('v_mag[npM:]', np.linalg.norm(v_rotational[npM:, :], axis=1))



    # def test_computeVaxial(self):
    #     npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(self.drone)
    #     total_colloc_points = computeCollocationPoints(self.drone, npM, npS, main_NB, small_NB, main_n, small_n)
    #     n_azimuth, n_origin = computeAzimuthAndOrigin(self.drone, npM, npS, main_NB, collocN)

    #     Omega = np.zeros((collocN, 1))
    #     Omega[:npM] = self.drone.main_prop.RPM*2*np.pi/60
    #     Omega[npM:] = self.drone.small_props[0].RPM*2*np.pi/60
    #     v_rotational = computeVrotational(self.drone, total_colloc_points, Omega, n_azimuth, n_origin, npM, npS, main_NB)
    #     v_axial = computeVaxial(v_rotational, n_azimuth, total_colloc_points, n_origin)

    #     print(v_axial[:npM])
    #     np.testing.assert_allclose(
    #         v_axial[:npM],
    #         -np.ones((npM)),
    #         rtol=0,
    #         atol=1e-6,
    #     )
    #     print(v_axial[npM:npM+3])
    #     np.testing.assert_allclose(
    #         v_axial[npM:npM+3],
    #         -np.array(v_rotational[npM:npM+3, 0]),
    #         rtol=0,
    #         atol=1e-6,
    #     )

    # def test_computeVtangential(self):
    #     npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(self.drone)
    #     total_colloc_points = computeCollocationPoints(self.drone, npM, npS, main_NB, small_NB, main_n, small_n)
    #     n_azimuth, n_origin = computeAzimuthAndOrigin(self.drone, npM, npS, main_NB, collocN)

    #     Omega = np.zeros((collocN, 1))
    #     Omega[:npM] = self.drone.main_prop.RPM*2*np.pi/60
    #     Omega[npM:] = self.drone.small_props[0].RPM*2*np.pi/60
    #     v_rotational = computeVrotational(self.drone, total_colloc_points, Omega, n_azimuth, n_origin, npM, npS, main_NB)
    #     v_tangential = computeVtangential(v_rotational, n_azimuth, total_colloc_points, n_origin)

    #     # m b1
    #     np.testing.assert_allclose(
    #         v_tangential[:3],
    #         v_rotational[:3, 0],
    #         rtol=0,
    #         atol=1e-6,
    #     )

    #     # s1 b1
    #     np.testing.assert_allclose(
    #         v_tangential[npM:npM+4],
    #         np.array([31.4159265359, 52.35987755983, 73.30382858376, 94.247779607]),
    #         rtol = 0,
    #         atol=1e-6,
    #     )

    #     # s2 b1 
    #     np.testing.assert_allclose(
    #         v_tangential[npM + npS: npM + npS + 4],
    #         np.array([31.4159265359, 52.35987755983, 73.30382858376, 94.247779607]),
    #         rtol = 0,
    #         atol=1e-6,
    #     )

    #     # s3 b1 
    #     np.testing.assert_allclose(
    #         v_tangential[npM + 2*npS: npM + 2*npS + 4],
    #         np.array([31.4159265359, 52.35987755983, 73.30382858376, 94.247779607]),
    #         rtol = 0,
    #         atol=1e-6,
    #     )
    
    # def test_computeInflowAndAlpha(self):
    #     # it is assumed that w = -np.ones((total_colloc_points.shape[0], 1))

    #     npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(self.drone)
    #     total_colloc_points = computeCollocationPoints(self.drone, npM, npS, main_NB, small_NB, main_n, small_n)
    #     n_azimuth, n_origin = computeAzimuthAndOrigin(self.drone, npM, npS, main_NB, collocN)

    #     twist = np.ones(collocN)*10

    #     Omega = np.zeros((collocN, 1))
    #     Omega[:npM] = self.drone.main_prop.RPM*2*np.pi/60
    #     Omega[npM:] = self.drone.small_props[0].RPM*2*np.pi/60
    #     v_rotational = computeVrotational(self.drone, total_colloc_points, Omega, n_azimuth, n_origin, npM, npS, main_NB)
    #     v_axial = computeVaxial(v_rotational, n_azimuth, total_colloc_points, n_origin)
    #     v_tangential = computeVtangential(v_rotational, n_azimuth, total_colloc_points, n_origin)
    #     print('v_rotationla[:npM, :]', v_rotational[:npM, :])
    #     print('v_axial[:npM]', v_axial[:npM])
    #     print('v_tangential[:npM]', v_tangential[:npM])

    #     inflowangle, alpha = computeInflowAndAlpha(v_tangential, v_axial, twist)

    #     print('inflowangle[:npM]*180/np.pi', inflowangle[:npM]*180/np.pi)
    #     print('alpha', alpha[:npM])

    #     print('v_rotationla[npM:npM+3, :]', v_rotational[npM:npM+3, :])
    #     print('v_axial[npM:npM+3]', v_axial[npM:npM+3])
    #     print('v_tangential[npM:npM+3]', v_tangential[npM: npM+3])

    #     print('inflowangle[:npM]*180/np.pi', inflowangle[npM: npM+3]*180/np.pi)
    #     print('alpha', alpha[npM: npM+3])

    # def test_computeForces(self):
    #     npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(self.drone)
    #     total_colloc_points = computeCollocationPoints(self.drone, npM, npS, main_NB, small_NB, main_n, small_n)
    #     n_azimuth, n_origin = computeAzimuthAndOrigin(self.drone, npM, npS, main_NB, collocN)

    #     twist = np.ones(collocN)*10

    #     Omega = np.zeros((collocN, 1))
    #     Omega[:npM] = self.drone.main_prop.RPM*2*np.pi/60
    #     Omega[npM:] = self.drone.small_props[0].RPM*2*np.pi/60
    #     v_rotational = computeVrotational(self.drone, total_colloc_points, Omega, n_azimuth, n_origin, npM, npS, main_NB)
    #     v_axial = computeVaxial(v_rotational, n_azimuth, total_colloc_points, n_origin)
    #     v_tangential = computeVtangential(v_rotational, n_azimuth, total_colloc_points, n_origin)

    #     v_mag = np.sqrt(v_axial**2 + v_tangential**2)
    #     print('v_mag[:3]', v_mag[:3])
    #     chords = computeChords(self.drone, npM, collocN, main_NB, small_NB, main_n, small_n)
    #     inflowangle, alpha = computeInflowAndAlpha(v_tangential, v_axial, twist)

    #     Cl, Lift, Drag = computeForces(self.drone, v_tangential, v_axial, alpha, npM, npS, chords, main_NB, small_NB, main_n, small_n)

    #     np.testing.assert_allclose(
    #         Cl[:3].flatten(),
    #         np.array([0.49953, 0.824066, 0.92019]),
    #         rtol=0,
    #         atol=1e-2,
    #     )
    #     print('Lift[:3].flatten()', Lift[:3].flatten())
    #     np.testing.assert_allclose(
    #         Lift[:3].flatten(),
    #         np.array([1.015753, 8.0521047, 21.451732]),
    #         rtol=0,
    #         atol=1e-2,
    #     )
    #     print('Drag[:3].flatten()', Drag[:3].flatten())

if __name__ == '__main__':
    unittest.main()