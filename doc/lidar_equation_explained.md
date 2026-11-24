> Historical analysis below describes the pre-fix implementation. Surface reflection
> now uses normalized modified Phong: `rho*c*((1-ks)/pi + ks*(n+2)/(2*pi)*max(0,2*c*c-1)^n)`.
> `ks` replaces `specularity`, with `kd=1-ks`; reflectance is fractional everywhere
> except spectral-file input. Exponent zero defines a constant lobe. The outgoing
> cosine multiplies both components. Direction-independent response remains an
> empirical override. Regenerate version-1 documents/scenes as version 2.
> LAD-LUT findings below remain unresolved by this surface-only correction.

# The HELIOS lidar equation, BRDF, and LAD lookup tables

The LAD-LUT path uses a different scattering model, and HELIOS's “BRDF” is not quite the same quantity as a physical BRDF. This explanation compares `intensity_fwf.rst` with the current implementation and the Carlsson report cited in the repository.

Equations use Markdown math delimiters (`$...$` and `$$...$$`) and require a math-enabled Markdown viewer.

## What the lidar equation describes

The equation accounts for how much laser power reaches a target, how the target scatters it, and how much scattered light enters the receiver.

For subray $i$, the documentation gives

$$
P_{r,i}
= P_0s_i
\frac{D_r^2}{16A_iR^2}
\sigma_i\eta_{\mathrm{sys}}e^{-2a_eR}.
$$

The quantities are:

| Symbol | Meaning |
| --- | --- |
| $P_0s_i$ | Emitted power assigned to this subray |
| $A_i$ | Subray footprint area perpendicular to propagation |
| $\sigma_i$ | Effective scattering cross section |
| $D_r$ | Receiver aperture diameter |
| $R$ | Target distance |
| $\eta_{\mathrm{sys}}$ | System efficiency |
| $a_e$ | Atmospheric extinction coefficient |
| $e^{-2a_eR}$ | Atmospheric transmission over the outward and return paths |

The geometric terms can be separated as

$$
P_{r,i}
=
\underbrace{\frac{P_0s_i}{A_i}}_{\text{incident irradiance}}
\underbrace{\frac{\sigma_i}{4\pi}}_{\text{scattering toward receiver}}
\underbrace{\frac{\pi D_r^2/4}{R^2}}_{\text{receiver solid angle}}
\eta_{\mathrm{sys}}e^{-2a_eR}.
$$

For ordinary surfaces, HELIOS substitutes

$$
\sigma_i = 4\pi A_i F_H(\varphi),
$$

where $F_H$ is what the docs call $\mathrm{BRDF}_r$. This cancels $A_i$, giving

$$
\boxed{
P_{r,i}
= P_0s_i\frac{\pi D_r^2}{4R^2}
F_H(\varphi)\eta_{\mathrm{sys}}e^{-2a_eR}
}.
$$

This is implemented in [EnergyModel.cpp](../src/maths/EnergyModel.cpp).

This also explains the apparent $R^{-2}$ versus $R^{-4}$ difference. Holding the cross section fixed gives $R^{-4}$, because $A_i\propto R^2$. A surface filling the subray footprint has a cross section growing with that footprint, leaving $R^{-2}$. These statements hold for fixed sampling geometry and omit the additional atmospheric attenuation.

## What a BRDF actually is

BRDF means *bidirectional reflectance distribution function*. It describes how strongly a material reflects light from an incoming direction into an outgoing direction:

$$
f_r(\omega_i,\omega_o)
= \frac{\text{outgoing radiance}}{\text{incoming irradiance}},
\qquad [f_r] = \mathrm{sr}^{-1}.
$$

Reflectance tells you the total reflected fraction; the BRDF tells you its directional distribution. For an ideal Lambertian material with reflectance $\rho$,

$$
f_r = \frac{\rho}{\pi}.
$$

The $1/\pi$ normalizes the total reflected power. [PBRT's Lambertian model](https://pbr-book.org/4ed/Reflection_Models/Diffuse_Reflection) uses this definition.

HELIOS instead computes the angular response

$$
F_H(\varphi)
= \rho_H\left[
(1-k_s)\cos\varphi
+ k_s\left|\cos(2\varphi)\right|^{N_s}
\right].
$$

Here $k_s$ controls the specular contribution and $N_s$ its sharpness. With $k_s=0$, this becomes

$$
F_H(\varphi)=\rho_H\cos\varphi.
$$

That quantity already contains a geometric cosine and lacks the Lambertian $1/\pi$ normalization. Calling it an *angular reflectance response*, as the code does, is more precise.

For a footprint fully intercepted by a surface, conventional radiometry gives, in the monostatic geometry,

$$
P_{r,i}
= P_0s_i
\frac{\pi D_r^2}{4R^2}
f_r(\varphi,\varphi)\cos\varphi
\eta_{\mathrm{sys}}e^{-2a_eR}.
$$

Thus a physical Lambertian surface gives

$$
P_{r,i}
= P_0s_i
\frac{D_r^2}{4R^2}
\rho\cos\varphi
\eta_{\mathrm{sys}}e^{-2a_eR}.
$$

This comparison finds a factor of $\pi$ between the physical Lambertian result and HELIOS's diffuse formula **if the same fractional reflectance is supplied**. This is a derivation from the equations, rather than a claim that their reflectance conventions are interchangeable.

Absolute scaling also needs care because the docs show percentage-style material values, and [the MTL reader](../src/assetloading/MaterialsFileReader.cpp) assigns those values directly.

## Lambertian reflection and the factor of pi

A Lambertian surface has the same **radiance** in every outgoing direction. This approximates a matte surface. It does not send equal power into every solid angle: viewed obliquely, its projected area is smaller, so the power per solid angle decreases with the cosine.

For incident power already intercepted by a surface patch, the fraction scattered into a small outgoing solid angle is

$$
\frac{\mathrm dP_r}{P_{\mathrm{incident}}}
= f_r(\omega_i,\omega_o)\cos\theta_o\,\mathrm d\Omega_o.
$$

For each fixed incoming direction, total reflectance is therefore the cosine-weighted integral over the **outward hemisphere**, not the whole sphere:

$$
\rho(\omega_i)
= \int_{\Omega^+} f_r(\omega_i,\omega_o)
\cos\theta_o\,\mathrm d\Omega_o.
$$

For Lambertian reflection, $f_r$ is constant. With polar angle $\theta$ measured from the normal and azimuth $\alpha$,

$$
\mathrm d\Omega = \sin\theta\,\mathrm d\theta\,\mathrm d\alpha,
$$

and

$$
\begin{aligned}
\int_{\Omega^+}\cos\theta\,\mathrm d\Omega
&= \int_0^{2\pi}\int_0^{\pi/2}
\cos\theta\sin\theta\,\mathrm d\theta\,\mathrm d\alpha \\
&= 2\pi\times\frac12 = \pi.
\end{aligned}
$$

Consequently,

$$
\rho = \pi f_r,
\qquad
f_r = \frac{\rho}{\pi}.
$$

The hemisphere occupies $2\pi$ steradians, but the cosine weighting makes the normalization denominator $\pi$. This is the normalization used in [PBRT's diffuse-reflection implementation](https://pbr-book.org/4ed/Reflection_Models/Diffuse_Reflection).

### Implications for the code branches

The relevant function is `computeAngularReflectance()` in [EnergyModel.cpp](../src/maths/EnergyModel.cpp). Its output is an angular return response. To match physical radiometry, it would represent the BRDF multiplied by the outgoing cosine. For monostatic lidar, the outgoing angle equals the incidence angle:

$$
F_H(\varphi) = f_r(\omega_i,\omega_o)\cos\varphi.
$$

The three branches therefore need separate consideration:

| Branch | Interpretation and normalization |
| --- | --- |
| `specularity == 0` (after the direction-independent check) | Lambertian angular shape. A physical response is `reflectance * incidenceCosine / PI`. |
| `isDirectionIndependent` | Constant return response, which is not Lambertian. Dividing by pi alone does not make it Lambertian. |
| `specularity != 0` (after the direction-independent check) | Diffuse/specular mixture. Its diffuse component needs division by pi; its specular component needs normalization appropriate to its directional distribution. |

For the **pure diffuse branch**, assuming `reflectance` is a fraction between zero and one, the corresponding formula would be:

```cpp
if (mat.specularity == 0)
    return mat.reflectance * incidenceCosine / PI;
```

This is a proposed physical normalization, not a description of a change already made to the implementation.

For the **direction-independent response**, making it Lambertian would also require adding the cosine, defeating the flag's existing purpose. It may be useful as an empirical return model, but its parameter should not automatically be interpreted as hemispherical reflectance.

For the **specular mixture**, dividing the entire expression by pi would rescale it, but would not establish energy conservation. The current expression specifies only the backscatter response as incidence varies. To normalize a physical BRDF, its distribution over **all outgoing directions for each fixed incoming direction** must be specified. Integrating the backscatter curve while varying incidence is a different operation.

The missing division by pi is directly demonstrable for the diffuse branch under the fractional-reflectance assumption. It is **not a universal normalization fix for all three branches**.

## What changes with LAD LUT

LAD means *leaf angle distribution*. It represents the orientations of unresolved leaves statistically, rather than using one surface normal.

The lookup-table path computes

$$
\sigma_{\mathrm{LUT}}
= \frac{p\,G(\mathbf d)}{2\pi},
$$

where $p$ is `PadBVTotal` and $G(\mathbf d)$ is the interpolated directional lookup value. See [LadLut.cpp](../src/maths/LadLut.cpp).

This value is inserted directly into the external-cross-section equation:

$$
P_{r,i}
= P_0s_i
\frac{D_r^2}{16A_iR^2}
\frac{p\,G(\mathbf d)}{2\pi}
\eta_{\mathrm{sys}}e^{-2a_eR}.
$$

Consequently:

- Material reflectance, specularity, and the Phong response are bypassed.
- Directional dependence comes from the LUT.
- The footprint area does not cancel.
- In transmittive voxel mode, the same value also controls the probability of interception.

There is a dimensional inconsistency worth investigating here: `PadBVTotal` is documented as plant area density, in $\mathrm{m}^{-1}$. For dimensionless $G$, the LUT result is an extinction coefficient, also in $\mathrm{m}^{-1}$, whereas the radar equation expects a cross section in $\mathrm{m}^2$.

Its use for random interception distances is dimensionally sensible:

$$
s = -\frac{\ln U}{\sigma_{\mathrm{LUT}}},
\qquad
P(\text{pass through length }L)
= e^{-\sigma_{\mathrm{LUT}}L},
$$

where $U$ is a uniform random variate between zero and one.

Its reuse as a scattering cross section does not follow from that. This is visible in [DetailedVoxel.cpp](../src/scene/primitives/DetailedVoxel.cpp) and the [return-amplitude branch](../src/scanner/detector/FullWaveformPulseRunnable.cpp). It should be treated as an existing implementation convention, not a physically equivalent replacement for the surface BRDF.

## How Carlsson differs

Assuming the intended reference is the report cited in the repository, Carlsson et al.'s Eq. 2.27 has the structure

$$
P_r
= \eta_{\mathrm{syst}}
\frac{E_p}{2\tau}
\rho_C
\frac{\pi r_a^2}{R^2}
e^{-2\sigma_{\mathrm{aer}}R}.
$$

Here $r_a$ is aperture **radius**, while HELIOS uses diameter; $\rho_C$ denotes their BRDF. They also use a different angular model, Eq. 2.10:

$$
\rho_C(\theta)
= \frac{A}{\cos^6\theta}
\exp\!\left[-\frac{\tan^2\theta}{s^2}\right]
+ B\cos^m\theta.
$$

The symbols $A$, $B$, $s$, and $m$ here are parameters of Carlsson's scattering model; they are distinct from the subray area, share, and random distance used above. This is not HELIOS's Phong expression. See [Carlsson et al., sections 2.2.2 and 2.5.2](https://www.foi.se/rest-api/report/FOI-R--0163--SE).

After converting radius to diameter, the receiver geometry agrees:

$$
\pi r_a^2 = \frac{\pi D_r^2}{4}.
$$

The meaningful differences are the pulse-amplitude convention, the angular scattering model, and BRDF normalization. Some differences are notation, but the surface-response definitions—and especially the LAD-LUT path—require more than algebraic rearrangement to reconcile.
